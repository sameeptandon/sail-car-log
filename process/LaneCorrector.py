#!/usr/bin/python
# -*- coding: utf-8 -*-
# Usage: LaneCorrector.py folder/ cam.avi raw_data.npz multilane_points.npz

from ArgParser import parse_args
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from Q50_config import LoadParameters
from VtkRenderer import VtkPointCloud, VtkBoundingBox, VtkText, VtkImage
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.distance import cdist
import sys
from transformations import euler_from_matrix
import vtk
from VideoReader import *
from ColorMap import heatColorMapFast
from MapReproject import lidarPtsToPixels
import math
from collections import deque
from pympler.asizeof import *


def vtk_transform_from_np(np4x4):
    vtk_matrix = vtk.vtkMatrix4x4()
    for r in range(4):
        for c in range(4):
            vtk_matrix.SetElement(r, c, np4x4[r, c])
    transform = vtk.vtkTransform()
    transform.SetMatrix(vtk_matrix)
    return transform


def get_transforms(args):
    """ Gets the IMU transforms for a run """
    gps_reader = GPSReader(args['gps'])
    gps_data = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(gps_data)
    return imu_transforms


def load_ply(ply_file):
    """ Loads a ply file and returns an actor """
    reader = vtk.vtkPLYReader()
    reader.SetFileName(ply_file)
    reader.Update()

    ply_mapper = vtk.vtkPolyDataMapper()
    ply_mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(ply_mapper)
    return actor


class Change (object):

    def __init__(self, selection, vector):
        self.selection = selection
        self.vector = vector

    def __str__(self):
        return '(%d, %d) %s' % (self.selection.lane, self.selection.idx,
                                self.vector)

    def performChange(self, direction=1):
        self.selection.move(direction * np.array(self.vector))
        self.selection.lowlight()


class BigChange (Change):

    def performChange(self, direction=1):
        for i in xrange(len(self.selection)):
            self.selection[i].move(direction * np.array(self.vector[i]))
            if direction == 1:
                self.selection[i].lowlight()
            else:
                self.selection[i].highlight()


class DeleteChange (Change):

    def __init__(self, selection, removed_points, lane):
        self.removed_points = removed_points
        self.lane = lane
        super(DeleteChange, self).__init__(selection, None)

    def performChange(self, direction=1):
        if direction == 1:
            self.selection.undelete(self.removed_points, self.lane)
        else:
            self.selection.delete()


class Undoer:

    def __init__(self):
        num_edits = 1000
        self.undo_queue = deque(maxlen=num_edits)
        self.redo_queue = deque(maxlen=num_edits)

    def __str__(self):
        s = 'undo:\n'
        for i in self.undo_queue:
            s += '\t' + str(i) + '\n'
        s += 'redo:\n'
        for i in self.redo_queue:
            s += '\t' + str(i) + '\n'
        return s

    def undo(self):
        try:
            undo = self.undo_queue.pop()
            self.redo_queue.append(undo)
            undo.performChange()
            print self.getSize(), 'MB'
        except IndexError:
            print 'Undo queue empty!'
            pass

    def redo(self):
        try:
            redo = self.redo_queue.pop()
            self.undo_queue.append(redo)
            redo.performChange(-1)
            print self.getSize(), 'MB'
        except IndexError:
            print 'Redo queue empty!'
            pass

    def addChange(self, change):
        try:
            # Try to append
            self.undo_queue.append(change)
        except IndexError:
            # Remove the oldest change
            self.undo_queue.popleft()
            # Add the new change
            self.undo_queue.append(change)
        self.redo_queue.clear()
        print self.getSize(), 'MB'

    def flush(self):
        self.redo_queue.clear()
        self.undo_queue.clear()

    def getSize(self):
        u = sum([asizeof(x) for x in self.undo_queue])
        r = sum([asizeof(x) for x in self.redo_queue])
        s = asizeof(self)
        return (u + r + s) / 2. ** 20


class Point:

    def __init__(self, actor, idx, blockworld):
        self.actor = actor
        self.blockworld = blockworld

        self.data = self.actor.GetMapper().GetInput()
        self.lane = self.getLane()
        if self.lane != -1:
            self.pos = self.blockworld.lane_clouds[self.lane].xyz
            self.color = self.blockworld.lane_clouds[self.lane].intensity
        else:
            self.pos = self.blockworld.raw_cloud.xyz
            self.color = self.blockworld.raw_cloud.intensity

        self.idx = idx

        self.start_pos = self.getPosition()

    def getLane(self):
        lane_actors = self.blockworld.lane_actors
        if self.actor in lane_actors:
            return lane_actors.index(self.actor)
        elif self.actor == self.blockworld.raw_actor:
            return -1
        raise RuntimeError('Could not find lane')

    def getPosition(self, offset=0):
        idx = min(self.idx + offset, self.pos.shape[0] - 1)
        return np.array(self.pos[idx,:])

    def selectExtreme(self):
        if self.idx > self.pos.shape[0] / 2:
            self.idx = self.pos.shape[0] - 1
        else:
            self.idx = 0

    def isCloser(self, other_point):
        pos = self.getPosition()
        other_pos = other_point.getPosition()
        return np.linalg.norm(pos) < np.linalg.norm(other_pos)

    def __str__(self):
        return '%d, %d: %s' % (self.lane, self.idx, self.getPosition())


class Selection:
    # Moving points
    Symmetric = 'symmetric'
    # Selects all points
    All = 'all'
    # Deleting points
    Delete = 'delete'
    # Adding points
    Append = 'append'
    Fork = 'fork'
    Copy = 'copy'
    Join = 'join'

    def __init__(self, parent, actor, idx, mode=Symmetric, end_idx=-1,
                 join_lane_actor=None):
        self.parent = parent
        self.blockworld = parent.parent

        # Selection mode
        self.mode = mode

        self.point = Point(actor, idx, self.blockworld)
        # By default make the points the same object
        self.end_point = self.point

        # Change this value to allow the copy to run
        self.copy_ready = False

        if self.mode == Selection.Symmetric:
            assert end_idx != -1
            self.end_point = Point(actor, end_idx, self.blockworld)

        elif self.mode == Selection.All:
            self.end_point = Point(actor, self.point.pos.shape[0] - 1,
                                   self.blockworld)

        elif self.mode in [Selection.Delete, Selection.Copy]:
            if end_idx != -1:
                self.end_point = Point(actor, end_idx, self.blockworld)

        elif self.mode in [Selection.Append, Selection.Fork]:
            if self.mode == Selection.Append:
                self.point.selectExtreme()
            if end_idx != -1:
                self.end_point = Point(self.blockworld.raw_actor, end_idx,
                                       self.blockworld)

        elif self.mode == Selection.Join:
            self.point.selectExtreme()
            if end_idx != -1 and join_lane_actor != None:
                self.end_point = Point(join_lane_actor, end_idx,
                                       self.blockworld)
                self.end_point.selectExtreme()

        else:
            raise RuntimeError('Bad selection mode: ' + self.mode)

        # Make sure the start point always comes before the end point
        if self.mode in [Selection.Symmetric, Selection.Delete, Selection.Copy]:
            # Symmetric, Delete, and Copy selections can use point idx because
            # all points are on the same actor
            if self.end_point.idx < self.point.idx:
                self.end_point, self.point = self.point, self.end_point
        else:
            # All other modes must use distance from the origin
            if self.end_point.isCloser(self.point):
                # Make sure the end point is not the raw points
                if self.end_point.actor != self.blockworld.raw_actor:
                    self.end_point, self.point = self.point, self.end_point

    def isSelected(self):
        if self.mode == Selection.Symmetric:
            return True
        else:
            return self.end_point != self.point

    def getPosition(self, offset=0):
        return self.point.getPosition(offset)

    def getStart(self):
        if self.mode == Selection.Symmetric:
            region = self.end_point.idx - self.point.idx
            return max(self.point.idx - region, 0)
        else:
            return self.point.idx

    def getEnd(self):
        return min(self.end_point.idx + 1, self.point.pos.shape[0])

    def nextPoint(self):
        return xrange(self.getStart(), self.getEnd())

    def highlight(self):
        self.setColor(self.blockworld.num_colors)

    def lowlight(self):
        self.setColor(self.point.lane)

    def setColor(self, color):
        self.point.color[[i for i in self.nextPoint()]] = color
        self.point.data.Modified()

    def move(self, vector):
        """ Vector is a change for the idx point. All other points in the
        selection region will move as well """

        points = [p for p in self.nextPoint()]
        weights = np.array([self.getWeight(p) for p in points])
        weights = np.tile(weights, (vector.shape[0], 1)).transpose()
        vector = np.tile(np.array(vector), (weights.shape[0], 1))

        self.point.pos[points,:] += weights * vector
        self.point.data.Modified()

    def getWeight(self, p):
        region = self.end_point.idx - self.point.idx
        alpha = abs(self.point.idx - p) / float(region)
        return (math.cos(alpha * math.pi) + 1) / 2.

    def delete(self):
        # Create a new lane actor
        lane = self.point.lane
        start = self.getStart()
        end = self.getEnd()
        new_lane = self.point.pos[end:]
        old_lane = self.point.pos[:start]
        del_segment = self.point.pos[start:end]

        if old_lane.shape[0] == 0 and new_lane.shape[0] == 0:
            # Delete the whole lane
            self.blockworld.removeLane(self.point.lane)
            self.point = None
            self.end_point = None
        elif new_lane.shape[0] == 0 or old_lane.shape[0] == 0:
            # If we are at the beginning or end
            at_beginning = new_lane.shape[0] > 0
            # Choose the segment that has points
            pts = new_lane if at_beginning else old_lane

            # Replace the lane
            lane_actor = self.blockworld.addLane(pts, self.point.lane,
                                                 replace=True)

            if at_beginning:
                self.point = Point(lane_actor, 0, self.blockworld)
                self.end_point = None
            else:
                self.point = None
                self.end_point = Point(lane_actor, start, self.blockworld)
        else:
            # Replace the old lane
            old_lane_actor = self.blockworld.addLane(old_lane, self.point.lane,
                                                     replace=True)
            # Add the new lane
            new_lane_actor = self.blockworld.addLane(new_lane, self.point.lane
                                                     + 1, replace = False)

            self.point = Point(old_lane_actor, start, self.blockworld)
            self.end_point = Point(new_lane_actor, 0, self.blockworld)

        return del_segment, lane

    def undelete(self, points, lane):
        if self.point == None and self.end_point == None:
            # Undeleting an entire lane
            undeleted_lane = self.blockworld.addLane(points, lane)
            self.point = Point(undeleted_lane, 0, self.blockworld)
            self.end_point = Point(undeleted_lane, len(points) - 1,
                                   self.blockworld)

        elif self.point == None or self.end_point == None:
            # Deleting from the end/beginning of the lane
            if self.point:
                # Add points onto the front
                data = np.concatenate((points, self.point.pos), axis=0)
            else:
                # Add points to the back
                data = np.concatenate((self.end_point.pos, points), axis=0)

            undeleted_lane = self.blockworld.addLane(data, lane,
                                                     replace=True)
            # Update the selection points
            if self.point:
                self.point = Point(undeleted_lane, 0, self.blockworld)
                self.end_point = Point(undeleted_lane, len(points) - 1,
                                                     self.blockworld)
            else:
                self.point = Point(undeleted_lane, self.end_point.idx,
                                                     self.blockworld)
                self.end_point = Point(undeleted_lane, self.end_point.idx +
                                                     len(points) - 1,
                                                     self.blockworld)
        else:
            # Undeleting from the middle of the lane
            data = np.concatenate((self.point.pos, points, self.end_point.pos),
                                  axis=0)

            undeleted_lane = self.blockworld.addLane(data, self.point.lane,
                                                     replace=True)
            self.blockworld.removeLane(self.end_point.lane)

            self.point = Point(undeleted_lane, self.point.idx, self.blockworld)
            self.end_point = Point(undeleted_lane,
                                   self.point.idx + len(points) - 1,
                                   self.blockworld)


    def append(self):
        if self.isSelected() and self.mode in [Selection.Append,
                                               Selection.Fork]:
            _, new_pts = self.interpolate(self.point, self.end_point)
            if self.mode == Selection.Append:
                # Check to see if we are adding to the end or the beginning
                if self.point.idx > 0:
                    data = np.append(self.point.pos, new_pts, axis=0)
                else:
                    data = np.append(
                        np.flipud(new_pts), self.point.pos, axis=0)
                self.blockworld.addLane(data, self.point.lane, replace=True)
            else:
                data = np.array(new_pts)
                self.blockworld.addLane(data)

    def join(self):
        if self.isSelected() and self.mode == Selection.Join:
            data, _ = self.interpolate(self.point, self.end_point)
            self.blockworld.addLane(data, self.point.lane, replace=True)
            self.blockworld.removeLane(self.end_point.lane)

    def interpolate(self, p1, p2):
        start = p1.pos[p1.idx, :3]
        end = p2.pos[p2.idx, :3]
        vector = end - start
        norm = np.linalg.norm(vector)
        n_vector = vector / norm

        new_pts = []
        step = 0.5

        for i in np.arange(step, norm, step):
            new_pts.append(start + n_vector * i)

        data = np.concatenate((p1.pos, np.array(new_pts), p2.pos), axis=0)

        return (data, np.array(new_pts))

    def copy(self, ground_idx):
        ground_pos = self.blockworld.raw_cloud.xyz[ground_idx,:]
        points = [p for p in self.nextPoint()]
        data = self.point.pos[points,:]
        # Translate points to origin, then to clicked point
        data += ground_pos - data[0,:]

        self.blockworld.addLane(data)

    def __str__(self):
        return '%s, %s' % (self.point, self.end_point)


class LaneInteractorStyle (vtk.vtkInteractorStyleTrackballCamera):

    def __init__(self, iren, ren, parent):
        self.iren = iren
        self.ren = ren
        self.parent = parent
        self.picker = vtk.vtkPointPicker()
        self.picker.SetTolerance(0.005)
        self.iren.SetPicker(self.picker)

        self.undoer = Undoer()

        self.moving = False
        self.selection = None

        self.mode = 'edit'

        self.num_to_move = 50

        self.SetMotionFactor(10.0)

        self.AutoAdjustCameraClippingRangeOff()
        self.ren.GetActiveCamera().SetClippingRange(0.01, 500)

        self.AddObserver('MouseWheelForwardEvent', self.MouseWheelForwardEvent)
        self.AddObserver(
            'MouseWheelBackwardEvent', self.MouseWheelBackwardEvent)

        self.AddObserver('RightButtonPressEvent', self.RightButtonPressEvent)
        self.AddObserver(
            'RightButtonReleaseEvent', self.RightButtonReleaseEvent)
        self.AddObserver('LeftButtonPressEvent', self.LeftButtonPressEvent)
        self.AddObserver('LeftButtonReleaseEvent', self.LeftButtonReleaseEvent)
        self.AddObserver('MouseMoveEvent', self.MouseMoveEvent)

        # Add keypress event
        self.AddObserver('CharEvent', self.KeyHandler)

    def MouseWheelForwardEvent(self, obj, event):
        self.OnMouseWheelForward()

    def MouseWheelBackwardEvent(self, obj, event):
        self.OnMouseWheelBackward()

    def RightButtonPressEvent(self, obj, event):
        x, y = self.iren.GetEventPosition()
        self.FindPokedRenderer(x, y)

        # Only allow rotation for the cloud camera
        if self.GetCurrentRenderer() == self.ren:
            self.OnLeftButtonDown()

    def RightButtonReleaseEvent(self, obj, event):
        self.OnLeftButtonUp()

    def LeftButtonPressEvent(self, obj, event):
        x, y = self.iren.GetEventPosition()
        self.picker.Pick(x, y, 0, self.ren)
        idx = self.picker.GetPointId()
        actor = self.picker.GetActor()
        if idx >= 0:
            if self.mode == 'edit':
                self.lowlightAll()
                self.selection = Selection(self, actor, idx,
                                           Selection.Symmetric,
                                           idx + self.num_to_move)
                self.moving = True

            elif self.mode in [Selection.Delete, Selection.Copy]:
                if self.selection == None:
                    self.selection = Selection(self, actor, idx, self.mode)
                else:
                    # Make sure we are selecting a point from the same lane
                    if self.selection.point.actor == actor:
                        self.selection.lowlight()

                        start = self.selection.point.idx
                        end = self.selection.end_point.idx
                        region = end - start

                        if abs(start - idx) < abs(end - idx):
                            start, end = idx, end
                        else:
                            start, end = start, idx
                        self.selection = Selection(self, actor, start,
                                                   self.mode, end)
                    self.selection.highlight()

                # If we are in copy mode and we have selected a lane point
                if self.mode == Selection.Copy and self.selection.copy_ready \
                   and actor == self.parent.raw_actor:
                    self.selection.copy(idx)
                    self.selection.lowlight()
                    self.KeyHandler(key='Escape')

            elif self.mode in [Selection.Append, Selection.Fork, Selection.Join]:
                if self.selection == None:
                    self.selection = Selection(self, actor, idx, self.mode)
                    self.selection.highlight()
                    if self.mode in [Selection.Append, Selection.Fork]:
                        self.togglePick(lane=False)
                else:
                    # Actors must be different for append, fork, and join
                    if actor != self.selection.point.actor:
                        join_lane_actor = actor if self.mode == Selection.Join else None

                        self.selection = Selection(self, self.selection.point.actor,
                                                   self.selection.point.idx, self.mode,
                                                   idx, join_lane_actor)
                        if self.mode in [Selection.Append, Selection.Fork]:
                            self.selection.append()
                        elif self.mode == Selection.Join:
                            self.selection.join()
                        self.KeyHandler(key='Escape')

    def LeftButtonReleaseEvent(self, obj, event):
        if self.moving and self.mode == 'edit':
            end_pos = self.selection.getPosition()
            vector = self.selection.point.start_pos - end_pos

            self.selection.lowlight()
            self.Render()

            change = Change(self.selection, vector)
            self.undoer.addChange(change)

            self.moving = False
            self.selection = None

    def MouseMoveEvent(self, obj, event):
        if self.moving and self.mode == 'edit':
            old_pos = self.selection.getPosition()
            x, y = self.iren.GetEventPosition()

            disp_obj_center = [0.] * 3
            new_pick_point = [0.] * 4
            self.ComputeWorldToDisplay(self.ren, old_pos[0], old_pos[1],
                                       old_pos[2], disp_obj_center)
            self.ComputeDisplayToWorld(self.ren, x, y, disp_obj_center[2],
                                       new_pick_point)

            new_pos = new_pick_point[:-1]
            new_pos[2] = old_pos[2]

            new_pos = np.array(new_pos)
            old_pos = np.array(old_pos)
            change = new_pos - old_pos

            # Move the point
            self.selection.move(change)
            self.selection.highlight()

            self.Render()

    def lowlightAll(self):
        for i in xrange(self.parent.num_lanes):
            self.lowlightLane(i)

    def lowlightLane(self, num):
        actor = self.parent.lane_actors[num]
        lane = Selection(self, actor, 0, Selection.All)
        lane.lowlight()
        self.Render()

    def togglePick(self, lane=True):
        self.parent.raw_actor.SetPickable(not lane)
        for l in self.parent.lane_actors:
            l.SetPickable(lane)

    def KeyHandler(self, obj=None, event=None, key=None):
        # Symbol names are declared in
        # GUISupport/Qt/QVTKInteractorAdapter.cxx
        # https://github.com/Kitware/VTK/
        if key == None:
            key = self.iren.GetKeySym()

        if key == 'q':
            # Todo: confirm quit
            self.iren.TerminateApp()
            return

        if not self.moving:
            if key == 'Escape':
                self.togglePick(lane=True)
                self.mode = 'edit'
                self.selection = None
                self.lowlightAll()

            if key == 'bracketright':
                # Increase the number of points selected
                self.num_to_move += 1
                self.mode = 'edit'

            elif key == 'bracketleft':
                # Decrease the number of points selected
                self.num_to_move = max(self.num_to_move - 1, 1)
                self.mode = 'edit'

            elif key in [str(i) for i in xrange(self.parent.num_lanes)]:

                self.mode = key

            elif key == 'd':
                if self.mode != Selection.Delete:
                    self.mode = Selection.Delete
                elif self.selection != None and self.selection.isSelected():
                    del_selection, lane = self.selection.delete()
                    change = DeleteChange(self.selection, del_selection, lane)
                    self.undoer.addChange(change)
                    self.KeyHandler(key='Escape')

            elif key == 'i':
                self.mode = 'insert'
            elif self.mode == 'insert':
                if key == 'a':
                    self.mode = Selection.Append
                elif key == 'f':
                    self.mode = Selection.Fork
                elif key == 'j':
                    self.mode = Selection.Join
                elif key == 'c':
                    self.mode = Selection.Copy
            elif self.mode == Selection.Copy:
                if key == 'c':
                    if self.selection:
                        self.selection.copy_ready = True
                        self.togglePick(lane=False)

            elif key == 's':
                file_name = 'out.npz'
                print 'Saved', file_name
                self.parent.exportData(file_name)

            elif key == 'f':
                if self.mode == 'edit':
                    print 'Fixing up all lanes'
                    self.parent.fixupLanes()
                elif self.mode in [str(i) for i in
                                   xrange(self.parent.num_lanes)]:
                    print 'Fixing up lane', self.mode
                    self.parent.fixupLane(int(self.mode))
                print 'Fixup finished'

            elif key == 'space':
                self.parent.running = not self.parent.running
                print 'Running' if self.parent.running else 'Paused'

            elif key == 'z':
                print 'Undo'
                self.undoer.undo()
                self.Render()

            elif key == 'y':
                print 'Redo'
                self.undoer.redo()
                self.Render()

            elif key == 'r':
                self.parent.record = not self.parent.record
                if self.parent.record:
                    print 'Recording multilane.avi'
                    self.startVideo()
                else:
                    print 'Done Recording'
                    self.closeVideo()

            elif key == 'Down':
                if not self.parent.running:
                    if self.parent.count > 0:
                        self.parent.count -= 1
                        self.parent.manual_change = -1

            elif key == 'Up':
                if not self.parent.running:
                    if not self.parent.finished():
                        self.parent.count += 1
                        self.parent.manual_change = 1

    def updateModeText(self):
        frame_num = self.parent.count
        tot_num = self.parent.video_reader.total_frame_count / self.parent.step
        mode = self.mode

        txt = '(%d/%d) ' % (frame_num, tot_num)
        if mode == 'edit':
            txt += 'Click to move lane | Move window [%d]' % self.num_to_move
        elif mode in [str(i) for i in xrange(self.parent.num_lanes)]:
            txt += 'Lane %s - All points' % mode
        elif mode == Selection.Delete:
            if self.selection == None:
                txt += 'Click to start delete segment'
            elif not self.selection.isSelected():
                txt = txt + 'Select another point to create delete segment'
            else:
                txt += '(d) - delete selected segment | click - ' + \
                       'change segment | (esc) - start over'
        elif mode == 'insert':
            txt += '(a) append | (f) fork | (c) copy | (j) join'
        elif mode == Selection.Append:
            if self.selection == None:
                txt += 'Appending (1/2). Select a lane'
            else:
                txt += 'Appending (2/2). Select a ground point'
        elif mode == Selection.Fork:
            if self.selection == None:
                txt += 'Forking (1/2). Select a point'
            else:
                txt += 'Forking (2/2). Select a ground point'
        elif mode == Selection.Join:
            if self.selection == None:
                txt += 'Joining (1/2). Select a lane'
            else:
                txt += 'Joining (2/2). Select a lane'
        elif mode == Selection.Copy:
            if self.selection == None:
                txt += 'Copy (1/3). Select a lane point to start copy'
            elif not self.selection.copy_ready:
                txt += 'Copy (2/3). Select a lane point to end copy. (c) ' + \
                       'to confirm selection'
            else:
                txt += 'Copy (3/3). Select ground point to begin copy'
        else:
            txt += 'All Lanes - %d' % self.num_to_move

        self.parent.selectModeActor.SetInput(txt)
        self.parent.selectModeActor.Modified()

    def Render(self):
        self.iren.GetRenderWindow().Render()

    def startVideo(self, video_name='multilane.avi'):
        self.win2img = vtk.vtkWindowToImageFilter()
        self.win2img.SetInput(self.parent.win)

        self.videoWriter = vtk.vtkFFMPEGWriter()
        self.videoWriter.SetFileName(video_name)
        self.videoWriter.SetInputConnection(self.win2img.GetOutputPort())
        self.videoWriter.SetRate(15)
        self.videoWriter.SetQuality(2)  # 2 is the highest
        self.videoWriter.SetBitRate(25000)
        self.videoWriter.SetBitRateTolerance(2000)
        self.videoWriter.Start()

    def writeVideo(self):
        self.win2img.Modified()
        self.videoWriter.Write()

    def closeVideo(self):
        self.videoWriter.End()


class Blockworld:

    def __init__(self):
        args = parse_args(sys.argv[1], sys.argv[2])

        self.start = 0
        self.step = 10
        self.end = self.step * 500
        self.count = 0

        ##### Grab all the transforms ######
        self.imu_transforms = get_transforms(args)
        self.cur_imu_transform = self.imu_transforms[0,:,:]

        self.trans_wrt_imu = self.imu_transforms[
            self.start:self.end:self.step, 0:3, 3]
        self.params = args['params']
        self.lidar_params = self.params['lidar']
        self.T_from_i_to_l = np.linalg.inv(self.lidar_params['T_from_l_to_i'])
        cam_num = args['cam_num']
        self.cam_params = self.params['cam'][cam_num - 1]

        # Whether to write video
        self.record = False
        # Is the flyover running
        self.running = True
        # Has the user changed the time
        self.manual_change = 0

        ###### Set up the renderers ######
        self.cloud_ren = vtk.vtkRenderer()
        self.cloud_ren.SetViewport(0, 0, 1.0, 1.0)
        self.cloud_ren.SetBackground(0, 0, 0)

        self.img_ren = vtk.vtkRenderer()
        self.img_ren.SetViewport(0.75, 0.0, 1.0, 0.25)
        # self.img_ren.SetInteractive(False)
        self.img_ren.SetBackground(0.1, 0.1, 0.1)

        self.win = vtk.vtkRenderWindow()
        self.win.StereoCapableWindowOff()
        self.win.AddRenderer(self.cloud_ren)
        self.win.AddRenderer(self.img_ren)
        self.win.SetSize(800, 400)

        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren.SetRenderWindow(self.win)

        ###### Cloud Actors ######
        print 'Adding raw points'
        raw_npz = np.load(sys.argv[3])
        pts = raw_npz['data']
        self.raw_cloud = VtkPointCloud(pts[:, :3], pts[:, 3])
        self.raw_actor = self.raw_cloud.get_vtk_cloud(zMin=0, zMax=100)
        self.raw_actor.GetProperty().SetPointSize(5)
        self.raw_actor.SetPickable(0)
        self.cloud_ren.AddActor(self.raw_actor)

        self.raw_kdtree = cKDTree(self.raw_cloud.xyz)

        print 'Loading interpolated lanes'
        npz = np.load(sys.argv[4])
        init_num_lanes = int(npz['num_lanes'])
        self.num_lanes = 0
        self.num_colors = 10

        self.lane_clouds = []
        self.lane_actors = []
        self.lane_kdtrees = []

        for i in xrange(init_num_lanes):
            interp_lane = npz['lane' + str(i)]
            self.addLane(interp_lane)

        # self.calculateZError(pts)

        print 'Adding car'
        self.car = load_ply('../mapping/viz/gtr.ply')
        self.car.SetPickable(0)
        self.car.GetProperty().LightingOff()
        self.cloud_ren.AddActor(self.car)

        # Use our custom mouse interactor
        self.interactor = LaneInteractorStyle(self.iren, self.cloud_ren, self)
        self.iren.SetInteractorStyle(self.interactor)

        # Tell the user which mode we are in
        selectMode = VtkText('Starting...', (10, 10))
        self.selectModeActor = selectMode.get_vtk_text()
        self.cloud_ren.AddActor(self.selectModeActor)

        ###### 2D Projection Actors ######
        self.video_reader = VideoReader(args['video'])
        self.img_actor = None

        ###### Add Callbacks ######
        print 'Rendering'

        self.iren.Initialize()

        # Set up time
        self.iren.AddObserver('TimerEvent', self.update)
        self.timer = self.iren.CreateRepeatingTimer(100)

        self.iren.Start()

    def addLane(self, data, lane=-1, replace=False):
        """ Appends a new lane to the dataset or replaces an index given by
        'lane'. If 'replace' is true, replace the index given by lane"""
        num_pts = data.shape[0]
        if lane == -1:
            lane_num = len(self.lane_clouds)
            old_actor = None
        elif replace == False:
            lane_num = lane
            old_actor = None
        else:
            lane_num = lane
            old_actor = self.lane_actors[lane]

        cloud = VtkPointCloud(data[:, :3], np.ones((num_pts)) *
                              (lane_num % self.num_colors))
        actor = cloud.get_vtk_cloud(zMin=0, zMax=self.num_colors)

        actor.GetProperty().SetPointSize(2)

        self.cloud_ren.RemoveActor(old_actor)
        self.cloud_ren.AddActor(actor)

        if replace and lane > -1:
            self.lane_clouds[lane_num] = cloud
            self.lane_actors[lane_num] = actor
            self.lane_kdtrees[lane_num] = cKDTree(cloud.xyz)
        elif lane > -1:
            self.lane_clouds.insert(lane_num, cloud)
            self.lane_actors.insert(lane_num, actor)
            self.lane_kdtrees.insert(lane_num, cKDTree(cloud.xyz))
            self.num_lanes += 1
        else:
            self.lane_clouds.append(cloud)
            self.lane_actors.append(actor)
            self.lane_kdtrees.append(cKDTree(cloud.xyz))
            self.num_lanes += 1

        return actor

    def removeLane(self, lane):
        actor = self.lane_actors[lane]
        cloud = self.lane_clouds[lane]
        tree = self.lane_kdtrees[lane]

        self.lane_clouds.remove(cloud)
        self.cloud_ren.RemoveActor(actor)
        self.lane_actors.remove(actor)
        self.lane_kdtrees.remove(tree)
        self.num_lanes -= 1

    def fixupLanes(self):
        self.interactor.lowlightAll()
        for l in xrange(self.num_lanes):
            self.fixupLane(l)
        self.interactor.Render()

    def fixupLane(self, num):
        lane = self.lane_clouds[num].xyz
        (d, idx) = self.raw_kdtree.query(lane, distance_upper_bound=0.25)

        mask = d < float('inf')
        close_lane = np.nonzero(mask)[0]
        close_raw = idx[mask]

        vectors = []
        selections = []
        for i in xrange(close_lane.shape[0]):
            vector = self.raw_cloud.xyz[close_raw[i]] - lane[close_lane[i]]
            sel = Selection(self.interactor, self.lane_actors[num],
                            close_lane[i], end_idx=close_lane[i] + 50)
            sel.move(vector)
            sel.highlight()

            vectors.append(vector)
            selections.append(sel)

        print '\tFixed lane %d changes: %d' % (num, len(vectors))

        big_change = BigChange(selections, vectors)
        self.interactor.undoer.addChange(big_change)

        self.interactor.Render()

    def calculateZError(self, pts):
        # Calculates median z-error of interpolated lanes to points
        tree = cKDTree(pts[:, :3])
        for num in xrange(self.num_lanes):
            lane = self.lane_clouds[num].xyz[:4910,:]
            z_dist = []
            for p in lane:
                (d, i) = tree.query(p)
                z_dist.append(abs(p[2] - pts[i, 2]))
            z_dist = np.array(z_dist)
            print 'Median Error in Lane %d: %f' % (num, np.median(z_dist))

    def getCameraPosition(self, t, focus=10):
        offset = np.array([-75.0, 0, 25.0]) / 4
        # Rotate the camera so it is behind the car
        position = np.dot(self.imu_transforms[t, 0:3, 0:3], offset)
        position += self.imu_transforms[t, 0:3, 3] + position

        # Focus 10 frames in front of the car
        focal_point = self.imu_transforms[t + focus * self.step, 0:3, 3]
        return position, focal_point

    def exportData(self, file_name):
        lanes = {}
        lanes['num_lanes'] = self.num_lanes
        for num in xrange(self.num_lanes):
            lane = self.lane_clouds[num].xyz[:, :3]
            offset = np.vstack((lane[1:,:], np.zeros((1, 3))))
            lane = np.hstack((lane, offset))
            lanes['lane' + str(num)] = lane

        np.savez(file_name, **lanes)

    def finished(self):
        t = self.start + self.step * self.count
        return t + 10 * self.step > self.video_reader.total_frame_count

    def update(self, iren, event):
        # Transform the car
        t = self.start + self.step * self.count
        cloud_cam = self.cloud_ren.GetActiveCamera()

        # If we have gone backwards in time we need use setframe (slow)
        if self.manual_change == -1:
            self.video_reader.setFrame(t - 1)

        if self.finished():
            return

        while self.video_reader.framenum <= t:
            (success, self.I) = self.video_reader.getNextFrame()

        # Copy the image so we can project points onto it
        I = self.I.copy()
        I = self.projectPointsOnImg(I, t)
        vtkimg = VtkImage(I)

        self.img_ren.RemoveActor(self.img_actor)
        self.img_actor = vtkimg.get_vtk_image()
        self.img_ren.AddActor(self.img_actor)

        if self.running or self.manual_change:
            # Set camera position to in front of the car
            position, focal_point = self.getCameraPosition(t)
            cloud_cam.SetPosition(position)
            cloud_cam.SetFocalPoint(focal_point)

            # Update the car position
            self.cur_imu_transform = self.imu_transforms[t,:,:]
            transform = vtk_transform_from_np(self.cur_imu_transform)
            transform.RotateZ(90)
            transform.Translate(-2, -3, -2)
            self.car.SetUserTransform(transform)

            # If the user caused a manual change, reset it
            self.manual_change = 0

        # Initialization
        if self.count == 0:
            cloud_cam.SetViewUp(0, 0, 1)
            self.img_ren.ResetCamera()
            # These units are pixels
            self.img_ren.GetActiveCamera().SetClippingRange(100, 100000)
            self.img_ren.GetActiveCamera().Dolly(1.75)

        # Update the little text in the bottom left
        self.interactor.updateModeText()

        if self.record:
            self.interactor.writeVideo()

        if self.running or self.count == 0:
            self.count += 1

        iren.GetRenderWindow().Render()

    def projectPointsOnImg(self, I, t):
        car_pos = self.imu_transforms[t, 0:3, 3]

        # Project the points onto the image
        for num in xrange(self.num_lanes):
            # Find the closest point
            tree = self.lane_kdtrees[num]
            (d, closest_idx) = tree.query(car_pos)

            # Find all the points nearby
            nearby_idx = np.array(tree.query_ball_point(car_pos, r=100.0))

            # Remove the points before the closest point
            nearby_idx = nearby_idx[nearby_idx > closest_idx]

            if nearby_idx.shape[0] > 0:
                lane = self.lane_clouds[num].xyz[nearby_idx, :3]

                if lane.shape[0] > 0:
                    pix, mask = lidarPtsToPixels(lane, self.imu_transforms[t,:,:],
                                                 self.T_from_i_to_l, self.cam_params)
                    intensity = np.ones((pix.shape[0], 1)) * num
                    heat_colors = heatColorMapFast(
                        intensity, 0, self.num_colors)
                    for p in range(4):
                        I[pix[1, mask]+p, pix[0, mask],:] = heat_colors[0,:,:]
                        I[pix[1, mask], pix[0, mask]+p,:] = heat_colors[0,:,:]
                        I[pix[1, mask]-p, pix[0, mask],:] = heat_colors[0,:,:]
                        I[pix[1, mask], pix[0, mask]-p,:] = heat_colors[0,:,:]

        return I

if __name__ == '__main__':
    Blockworld()
