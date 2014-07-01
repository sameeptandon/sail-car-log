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


class Change:

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
        except IndexError:
            print 'Undo queue empty!'
            pass

    def redo(self):
        try:
            redo = self.redo_queue.pop()
            self.undo_queue.append(redo)
            redo.performChange(-1)
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

    def flush(self):
        self.redo_queue.clear()
        self.undo_queue.clear()


class Selection:
    # Moving points
    symmetric = 0
    # Deleting points
    direct = 1
    # Adding points
    append = 2
    fork = 3
    new = 4
    join = 5

    def __init__(self, parent, actor, idx, mode=symmetric, end_idx=-1):
        self.parent = parent
        self.blockworld = parent.parent

        self.actor = actor

        # Selection mode
        self.mode = mode

        self.idx = idx
        self.lane = self.getLane()

        self.data = self.actor.GetMapper().GetInput()
        self.pos = self.blockworld.lane_clouds[self.lane].xyz
        self.color = self.blockworld.lane_clouds[self.lane].intensity

        self.startPos = self.getPosition()

        # The region of points are on either side of idx
        if self.mode == Selection.symmetric:
            self.region = parent.num_to_move

        # The region starts at idx and ends at end_idx
        elif self.mode == Selection.direct:
            # Make sure start is before end
            if end_idx >= 0:
                if self.idx > end_idx:
                    # Swap
                    self.idx, end_idx = end_idx, self.idx

                self.region = end_idx - self.idx + 1
            else:
                self.region = 0
        elif self.mode == Selection.append or self.mode == Selection.fork:
            self.region = 1
            self.idx = -1 if mode == Selection.append else idx

            self.raw_pos = self.blockworld.raw_cloud.xyz
            self.raw_idx = end_idx
        else:
            raise RuntimeError('Bad selection mode')

    def isSelected(self):
        if self.mode == Selection.symmetric:
            return True
        if self.mode == Selection.append or self.mode == Selection.fork:
            return self.raw_idx != -1
        else:
            return self.region != 0

    def getLane(self):
        lane_actors = self.blockworld.lane_actors
        if self.actor and self.actor in lane_actors:
            return lane_actors.index(self.actor)
        if self.mode == Selection.append:
            return None
        raise RuntimeError('Could not find lane')

    def getPosition(self, offset=0):
        return np.array(self.pos[self.idx + offset,:])

    def getStart(self):
        if self.mode == Selection.symmetric:
            return max(self.idx - self.region + 1, 0)
        else:
            return self.idx

    def getEnd(self):
        return min(self.idx + self.region, self.pos.shape[0])

    def nextPoint(self):
        return xrange(self.getStart(), self.getEnd())

    def highlight(self):
        self.setColor(self.blockworld.num_colors)

    def lowlight(self):
        self.setColor(self.lane)

    def setColor(self, color):
        self.color[[i for i in self.nextPoint()]] = color
        self.data.Modified()

    def move(self, vector):
        """ Vector is a change for the idx point. All other points in the
        selection region will move as well """

        points = [p for p in self.nextPoint()]
        weights = np.array([self.getWeight(p) for p in points])
        weights = np.tile(weights, (vector.shape[0], 1)).transpose()
        vector = np.tile(np.array(vector), (weights.shape[0], 1))

        self.pos[points,:] += weights * vector
        self.data.Modified()

    def delete(self):
        # Create a new lane actor
        new_lane = self.pos[:self.getStart()]
        old_lane = self.pos[self.getEnd():]

        if new_lane.shape[0] > 0:
            self.blockworld.addLane(self.pos[:self.getStart()])
        if old_lane.shape[0] > 0:
            # Replace insert a new actor into the old lane index
            self.blockworld.addLane(self.pos[self.getEnd():], self.lane)

        if old_lane.shape[0] == 0 and new_lane.shape[0] == 0:
            self.blockworld.removeLane(self.lane)

        self.parent.undoer.flush()
        
    def append(self):
        if self.isSelected() and self.mode == Selection.append \
           or self.mode == Selection.fork:
            start = self.pos[self.idx, :]
            end = self.raw_pos[self.raw_idx, :3]
            vector = end - start
            n_vector = vector / np.linalg.norm(vector)

            new_pts = []
            step = 0.5

            for i in np.arange(step, np.linalg.norm(vector), step):
                new_pts.append(start + n_vector * i)

            if self.mode == Selection.append:
                data = np.append(self.pos, np.array(new_pts), axis=0)
                self.blockworld.addLane(data, self.lane)
            else:
                data = np.array(new_pts)
                self.blockworld.addLane(data)

    def getWeight(self, p):
        alpha = abs(self.idx - p) / float(self.region)
        return (math.cos(alpha * math.pi) + 1) / 2.


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
                self.selection = Selection(self, actor, idx)
                self.moving = True
            elif self.mode == 'delete':
                if self.selection == None:
                    self.selection = Selection(self, actor, idx,
                                               Selection.direct)

                elif self.selection.actor == actor:
                    # Make sure we are selecting a point from the same lane
                    self.selection.lowlight()

                    start = self.selection.idx
                    region = self.selection.region
                    end = start + region

                    if abs(start - idx) < abs(end - idx):
                        start, end = idx, end
                    else:
                        start, end = start, idx
                    self.selection = Selection(self, actor, start,
                                               Selection.direct, end)
                self.selection.highlight()

            elif self.mode == 'append' or self.mode == 'fork':
                s_mode = Selection.append if self.mode == 'append' else Selection.fork
                if self.selection == None:
                    self.selection = Selection(self, actor, idx, s_mode)
                    self.selection.highlight()
                    self.togglePick(lane=False)
                else:
                    self.selection = Selection(self, self.selection.actor,
                                               self.selection.idx, s_mode, idx)
                    self.selection.append()
                    self.KeyHandler(key='Escape')

    def LeftButtonReleaseEvent(self, obj, event):
        if self.moving and self.mode == 'edit':
            endPos = self.selection.getPosition()
            vector = self.selection.startPos - endPos

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
        lane = Selection(self, self.parent.lane_actors[num], 0,
                         Selection.direct,
                         self.parent.lane_clouds[num].xyz.shape[0])
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
                if self.mode in ['append', 'fork', 'join', 'create']:
                    self.togglePick(lane=True)
                    self.mode = 'insert'
                else:
                    self.mode = 'edit'

                self.selection = None
                self.lowlightAll()

            if key == 'bracketright':
                # Increase the number of points selected
                self.num_to_move += 1
                self.mode = 'edit'

            elif key == 'bracketleft':
                # Decrease the number of points selected
                num = self.num_to_move - 1
                self.num_to_move = num if num > 0 else 1
                self.mode = 'edit'

            elif key in [str(i) for i in xrange(self.parent.num_lanes)]:
                self.mode = key

            elif key == 'd':
                if self.mode != 'delete':
                    self.mode = 'delete'
                elif self.selection != None and self.selection.isSelected():
                    self.selection.delete()
                    self.selection = None
                    self.mode = 'edit'

            elif key == 'i':
                self.mode = 'insert'
            elif self.mode == 'insert':
                if key == 'a':
                    self.mode = 'append'
                elif key == 'f':
                    self.mode = 'fork'
                elif key == 'j':
                    self.mode = 'join'
                elif key == 'c':
                    self.togglePick(lane=False)
                    self.mode = 'create'

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
        elif mode == 'delete':
            if self.selection == None:
                txt += 'Click to start delete segment'
            elif not self.selection.isSelected():
                txt = txt + 'Select another point to create delete segment'
            else:
                txt += '\'d\' - delete selected segment | click - ' + \
                    'change segment | \'esc\' - start over'
        elif mode == 'insert':
                txt += '(a) append | (f) fork | (c) create | (j) join'
        elif mode == 'append':
            if self.selection == None:
                txt += 'Select a lane to append to'
            else:
                txt += 'Select a ground point to create a lane'
        elif mode == 'fork':
            if self.selection == None:
                txt += 'Select a point to fork off of'
            else:
                txt += 'Select a ground point to create a lane'
        elif mode == 'create':
            if self.selection == None:
                txt += 'Select a ground point to start a lane'
            else:
                txt += 'Select a ground point to create a lane'
        elif mode == 'join':
            if self.selection == None:
                txt += 'Select a lane to start join'
            else:
                txt += 'Select a lane to end join'
        else:
            txt +='All Lanes - %d' % self.num_to_move

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

        # This will be created the first time we run fixup
        self.raw_kdtree = None

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

    def addLane(self, data, lane=-1):
        """ Appends a new lane to the dataset or replaces an index given by
        'lane' """
        num_pts = data.shape[0]
        if lane == -1:
            num_lanes = len(self.lane_clouds)
            old_actor = None
        else:
            num_lanes = lane
            old_actor = self.lane_actors[lane]

        cloud = VtkPointCloud(data[:, :3], np.ones((num_pts)) *
                              (num_lanes % self.num_colors))
        actor = cloud.get_vtk_cloud(zMin=0, zMax=self.num_colors)

        actor.GetProperty().SetPointSize(2)

        self.cloud_ren.RemoveActor(old_actor)
        self.cloud_ren.AddActor(actor)

        if lane == -1:
            self.lane_clouds.append(cloud)
            self.lane_actors.append(actor)
            self.lane_kdtrees.append(cKDTree(cloud.xyz))
            self.num_lanes += 1
        else:
            self.lane_clouds[lane] = cloud
            self.lane_actors[lane] = actor
            self.lane_kdtrees[lane] = cKDTree(cloud.xyz)

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

        if self.raw_kdtree == None:
            print '\tBuilding raw KD Tree, this may take a while...'
            self.raw_kdtree = cKDTree(self.raw_cloud.xyz)

        lane = self.lane_clouds[num].xyz
        (d, idx) = self.raw_kdtree.query(lane, distance_upper_bound=0.15)

        mask = d < float('inf')
        close_lane = np.nonzero(mask)[0]
        close_raw = idx[mask]

        vectors = []
        selections = []
        for i in xrange(close_lane.shape[0]):
            vector = self.raw_cloud.xyz[close_raw[i]] - lane[close_lane[i]]
            sel = Selection(self.interactor, self.lane_actors[num],
                            close_lane[i])
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
            imu_transform = self.imu_transforms[t,:,:]
            transform = vtk_transform_from_np(imu_transform)
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
