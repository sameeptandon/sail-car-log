#!/usr/bin/python
# -*- coding: utf-8 -*-
# Usage: GroundPlaneFitting

#!/usr/bin/python
# -*- coding: utf-8 -*-
# Usage: LaneCorrector.py folder/ cam.avi raw_data.npz multilane_points.npz

import bisect
from collections import deque
from colorsys import hsv_to_rgb
import glob
import math
import multiprocessing
import os
import sys
import time
import urllib

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.spatial import cKDTree
from scipy.signal import butter, filtfilt
import vtk

from ArgParser import parse_args
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from LidarTransforms import utc_from_gps_log_all
from MapReproject import lidarPtsToPixels
from VideoReader import VideoReader
from VtkRenderer import VtkPointCloud, VtkText, VtkImage


def vtk_transform_from_np(np4x4):
    vtk_matrix = vtk.vtkMatrix4x4()
    for r in range(4):
        for c in range(4):
            vtk_matrix.SetElement(r, c, np4x4[r, c])
    transform = vtk.vtkTransform()
    transform.SetMatrix(vtk_matrix)
    return transform


def get_transforms(args, mark='mark1'):
    """ Gets the IMU transforms for a run """
    gps_reader = GPSReader(args['gps_' + mark])
    gps_data = gps_reader.getNumericData()
    gps_times = utc_from_gps_log_all(gps_data)
    imu_transforms = IMUTransforms(gps_data)
    return imu_transforms, gps_data, gps_times


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


def load_gmaps(framelatlon):
    frame, latlon = framelatlon
    latlon_str = '%f,%f' % tuple(latlon)
    dirname, fname = get_gmap_fname(frame)
    url = ('http://maps.googleapis.com/maps/api/staticmap' +
           '?center=%s&zoom=19&size=400x400&maptype=satellite' +
           '&key=AIzaSyDNypM9M7HEdctMR4_hMo3jTadOwr-LR4Q') % \
        (latlon_str)
    try:
        os.mkdir(dirname)
        print 'Downloading Google map files...'
    except OSError:
        pass

    if not os.path.isfile(fname):
        f = open(fname, 'w')
        req = urllib.urlopen(url)
        f.write(req.read())


def get_gmap_fname(frame):
    dirname = sys.argv[1] + '/gmaps/'
    fname = dirname + str(frame) + '.jpg'
    return dirname, fname


class Change (object):

    def __init__(self, selection, vector):
        self.selection = selection
        self.vector = vector

    def __str__(self):
        return '(%d, %d) %s' % (self.selection.point.lane,
                                self.selection.point.idx,
                                self.vector)

    def performChange(self, direction=1):
        self.selection.move(direction * np.array(self.vector))
        self.selection.lowlight()


class BigChange (Change):

    def performChange(self, direction=1):
        start = self.selection.point.idx
        end = self.selection.end_point.idx + 1
        self.selection.point.pos[start:end, :] += direction * self.vector
        self.selection.lowlight()


class DeleteChange (Change):

    def __init__(self, selection, removed_points, lane):
        self.removed_points = removed_points
        self.lane = lane
        super(DeleteChange, self).__init__(selection, None)

    def performChange(self, direction=1):
        if direction == -1:
            self.selection.undelete(self.removed_points, self.lane)
        else:
            self.selection.delete()

    def __str__(self):
        return '(%d) %s...' % (self.lane, self.removed_points[0, :])


class InsertChange (DeleteChange):

    def performChange(self, direction=1):
        super(InsertChange, self).performChange(direction * -1)


class Undoer:

    def __init__(self, parent):
        self.parent = parent
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
            undo.performChange(-1)
            self.parent.parent.refreshLaneColors()
        except IndexError:
            print 'Undo queue empty!'

    def redo(self):
        try:
            redo = self.redo_queue.pop()
            self.undo_queue.append(redo)
            redo.performChange()
            self.parent.parent.refreshLaneColors()
        except IndexError:
            print 'Redo queue empty!'

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
        self.parent.KeyHandler(key='s')

    def flush(self):
        self.redo_queue.clear()
        self.undo_queue.clear()


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
        return np.array(self.pos[idx, :])

    def selectExtreme(self):
        if self.idx > self.pos.shape[0] / 2:
            self.idx = self.pos.shape[0] - 1
        else:
            self.idx = 0

    def isCloser(self, other_point):
        pos = self.getPosition()
        other_pos = other_point.getPosition()
        _, pos_idx = self.blockworld.imu_kdtree.query(pos)
        _, other_pos_idx = self.blockworld.imu_kdtree.query(other_pos)
        return pos_idx < other_pos_idx

    def dist(self, other):
        return np.linalg.norm(other.getPosition() - self.getPosition())

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
    New = 'new'
    # Fixup
    Fixup = 'fixup'

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
        self.copying = False
        self.deleted = False

        if self.mode == Selection.Symmetric:
            assert end_idx != -1
            self.end_point = Point(actor, end_idx, self.blockworld)

        elif self.mode == Selection.All:
            self.point = Point(actor, 0, self.blockworld)
            self.end_point = Point(actor, self.point.pos.shape[0] - 1,
                                   self.blockworld)

        elif self.mode in [Selection.Delete, Selection.Copy, Selection.Fixup]:
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

        elif self.mode == Selection.New:
            self.point = Point(actor, idx, self.blockworld)
            if end_idx > -1:
                self.end_point = Point(actor, end_idx, self.blockworld)
        else:
            raise RuntimeError('Bad selection mode: ' + self.mode)

        # Make Sure the start point always comes before the end point
        if self.mode in [Selection.Symmetric, Selection.Delete, Selection.Copy,
                         Selection.All, Selection.Fixup]:
            # Symmetric, Delete, and Copy, All selections can use point idx
            # because all points are on the same actor
            if self.end_point.idx < self.point.idx:
                self.end_point, self.point = self.point, self.end_point
        else:
            # All other modes must use distance from the origin
            if self.end_point.actor != self.blockworld.raw_actor or \
               self.end_point.actor == self.point.actor:
                if self.end_point.isCloser(self.point):
                    # Make sure the end point is not the raw points
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
        color = self.point.lane % self.blockworld.num_colors
        self.setColor(color)

    def refreshColor(self):
        color = self.point.lane % self.blockworld.num_colors
        self.setColor(color, True)

    def setColor(self, color, all_points=False):
        if all_points:
            self.point.color[:] = self.blockworld.colors[color]
        else:
            points = [i for i in self.nextPoint()]
            self.point.color[points] = self.blockworld.colors[color]
        self.point.data.Modified()

    def move(self, vector):
        """ Vector is a change for the idx point. All other points in the
        selection region will move as well """

        points = [p for p in self.nextPoint()]
        weights = np.array([self.getWeight(p) for p in points])
        weights = np.tile(weights, (vector.shape[0], 1)).transpose()
        vector = np.tile(np.array(vector), (weights.shape[0], 1))

        self.point.pos[points, :] += weights * vector
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
                                                     + 1, replace=False)

            self.point = Point(old_lane_actor, start, self.blockworld)
            self.end_point = Point(new_lane_actor, 0, self.blockworld)

            self.blockworld.refreshLaneColors()

        self.deleted = True
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

        self.blockworld.refreshLaneColors()
        self.deleted = False

    def append(self):
        if self.isSelected() and self.mode in [Selection.Append,
                                               Selection.Fork]:
            if self.point.dist(self.end_point) < 0.5:
                print 'Points were too close! Try a new point'
                return np.array([])

            new_pts = self.interpolate(self.point, self.end_point, all=False)

            if self.mode == Selection.Append:
                # Check to see if we are adding to the end or the beginning
                at_end = self.point.idx > 0
                if at_end:
                    data = np.append(self.point.pos, new_pts, axis=0)
                else:
                    data = np.append(
                        np.flipud(new_pts), self.point.pos, axis=0)
                lane = self.blockworld.addLane(data, self.point.lane,
                                               replace=True)

                if at_end:
                    self.point = Point(
                        lane, self.point.idx + 1, self.blockworld)
                else:
                    self.point = Point(lane, self.point.idx, self.blockworld)
                self.end_point = Point(lane, self.point.idx + len(new_pts) - 1,
                                       self.blockworld)
            else:
                if self.end_point.isCloser(self.point):
                    new_pts = new_pts[::-1]
                    at_end = False
                else:
                    at_end = True
                lane = self.blockworld.addLane(new_pts)

                self.point = Point(lane, 0, self.blockworld)
                self.end_point = Point(lane, len(new_pts) - 1, self.blockworld)

            # Choose which point to start the new append
            next_point = self.end_point if at_end else self.point
            return new_pts, next_point

    def join(self):
        if self.point.idx == 0 and self.end_point.idx == 0 or \
           self.point.idx > 0 and self.end_point.idx > 0:
            print 'Error: Joining from start to start or end to end'
            return None

        data, new_pts = self.interpolate(self.point, self.end_point)

        if new_pts.shape[0] == 0:
            return None

        # Don't let points connect if it will cause a discontenuity
        dist = np.linalg.norm(data, axis=1)
        diff = np.diff(dist)
        if np.any(diff < -.05 * np.max(dist)):
            print 'Error: Points too far apart'
            return None

        lane = self.blockworld.addLane(data, self.point.lane, replace=True)
        self.blockworld.removeLane(self.end_point.lane)

        self.point = Point(lane, self.point.idx + 1, self.blockworld)
        self.end_point = Point(lane, self.point.idx + len(new_pts) - 1,
                               self.blockworld)
        self.blockworld.refreshLaneColors()
        return new_pts

    def interpolate(self, p1, p2, all=True):
        start = p1.pos[p1.idx, :3]
        end = p2.pos[p2.idx, :3]
        vector = end - start
        norm = np.linalg.norm(vector)
        n_vector = vector / norm

        step = 0.5
        alpha = np.arange(step, norm, step)

        new_pts = start + \
            np.tile(n_vector, (len(alpha), 1)) * alpha[:, np.newaxis]

        if all:
            data = np.concatenate((p1.pos, new_pts, p2.pos), axis=0)
            return (data, new_pts)
        else:
            return new_pts

    def copy(self, ground_idx):
        ground_pos = self.blockworld.raw_cloud.xyz[ground_idx, :]
        points = [p for p in self.nextPoint()]
        data = self.point.pos[points, :]
        # Translate points to origin, then to clicked point
        data += ground_pos - data[0, :]

        lane = self.blockworld.addLane(data)

        self.point = Point(lane, 0, self.blockworld)
        self.end_point = Point(lane, len(data) - 1, self.blockworld)

        return data

    def new(self):
        if self.point.dist(self.end_point) < 0.5:
            print 'Error: Points were too close'
            return None

        new_pts = self.interpolate(self.point, self.end_point, all=False)
        lane = self.blockworld.addLane(new_pts)
        self.point = Point(lane, 0, self.blockworld)
        self.end_point = Point(lane, new_pts.shape[0] - 1, self.blockworld)

        return new_pts

    def calculateError(self):
        # Calculates median z-error of interpolated lanes to points
        lane = self.point.pos[self.point.idx:self.end_point.idx + 1]
        d, _ = self.blockworld.raw_kdtree.query(lane)
        return np.median(d)

    def fixup(self):
        lane = self.point.pos[self.point.idx:self.end_point.idx + 1, :]
        if lane.shape[0] < 20:
            print 'Lane was too small to smooth'
            return
        orig_lane = lane.copy()

        actor = self.point.actor

        err_start = self.calculateError()

        raw_kdtree = self.blockworld.raw_kdtree
        (d, idx) = raw_kdtree.query(lane, distance_upper_bound=0.25)

        mask = d < float('inf')
        close_lane = np.nonzero(mask)[0]
        close_raw = idx[mask]

        # Build the clusters by grouping indices
        clusters = []
        cluster = []
        for i, lane_idx in enumerate(close_lane):
            if i == len(close_lane) - 1:
                break
            cluster.append(i)
            if abs(close_lane[i] - close_lane[i + 1]) >= 2:
                clusters.append(cluster[len(cluster) / 2])
                cluster = []

        # Move the middle point in the cluster
        raw_pos = self.blockworld.raw_cloud.xyz[close_raw, :]
        lane_pos = lane[close_lane, :]
        lane[close_lane, :] += raw_pos - lane_pos

        # Sometimes the 0th point is very far away and it causes problems
        start = 1 if np.linalg.norm(lane[0, :] - lane[1,:]) > 1 else 0
        # Respace the points and try to smooth a little bit
        num = lane.shape[0]

        t = np.arange(start, num)

        # Weight points that are on lane points higher
        w = np.ones(lane.shape[0], dtype=np.float32)
        w[close_lane] = 2.
        w = w[start:]

        s_small = max(num / 100, 1)
        s_big = max(num / 50, 1)
        xinter = UnivariateSpline(t, lane[start:, 0], w, s=s_small)
        yinter = UnivariateSpline(t, lane[start:, 1], w, s=s_small)
        zinter = UnivariateSpline(t, lane[start:, 2], w, s=s_big)
        lane[start:, 0] = xinter(t)
        lane[start:, 1] = yinter(t)
        lane[start:, 2] = zinter(t)

        # Run a low pass filter across the point (0.9 Hz cutoff) -- magic
        # number
        # b, a = butter(4, 0.01, 'low')
        # f = filtfilt(b, a, self.point.pos, axis=0)
        # f = f[self.point.idx:self.end_point.idx + 1, :]
        # lane[start:, 2] =  f[:, 2]

        # t = np.arange(0, lane.shape[0])
        # plt.plot(t, orig_lane[:, 2], 'r--', t, lane[:, 2], 'g--')
        # plt.show()

        err_end = self.calculateError()
        print '\tFixed lane %d changes: %d Error: %f -> %f' % \
            (self.point.lane, close_lane.shape[0], err_start, err_end)

        actor = self.blockworld.lane_actors[self.point.lane]
        selection = Selection(self.parent, actor, self.point.idx,
                              Selection.Fixup, self.end_point.idx)
        print '\t', np.median(lane - orig_lane, axis=0)
        big_change = BigChange(selection, lane - orig_lane)
        self.parent.undoer.addChange(big_change)

        self.point.data.Modified()
        self.parent.Render()

    def __str__(self):
        return '%s, %s' % (self.point, self.end_point)


class LaneInteractorStyle (vtk.vtkInteractorStyleTrackballCamera):

    def __init__(self, iren, ren, parent):
        self.iren = iren
        self.ren = ren
        self.parent = parent
        self.picker = vtk.vtkPointPicker()
        self.picker.SetTolerance(0.003)
        self.iren.SetPicker(self.picker)

        self.undoer = Undoer(self)

        self.moving = False
        self.selection = None

        self.mode = 'edit'

        self.num_to_move = 50

        self.hover_lane = 0
        self.hover_point = 0
        self.highlighted_lanes = []

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

            elif self.mode in [Selection.Delete, Selection.Copy,
                               Selection.Fixup]:
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
                    self.selection.lowlight()

                    self.selection.copying = False
                    self.parent.removeLane(self.selection.point.lane)

                    pts = self.selection.copy(idx)
                    change = InsertChange(self.selection, pts,
                                          self.selection.point.lane)
                    self.undoer.addChange(change)
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
                        join_lane_actor = actor if self.mode == Selection.Join \
                            else None

                        self.selection = Selection(self, self.selection.point.actor,
                                                   self.selection.point.idx, self.mode,
                                                   idx, join_lane_actor)
                        self.selection.lowlight()
                        if self.mode in [Selection.Append, Selection.Fork]:
                            pts, next_point = self.selection.append()
                        elif self.mode == Selection.Join:
                            pts = self.selection.join()

                        if pts == None or pts.shape[0] == 0:
                            self.KeyHandler(key='Escape')
                            return

                        change = InsertChange(self.selection, pts,
                                              self.selection.point.lane)
                        self.undoer.addChange(change)

                        if self.mode in [Selection.Append, Selection.Fork]:
                            if self.mode == Selection.Fork:
                                self.mode = Selection.Append
                            self.selection = Selection(self, next_point.actor,
                                                       next_point.idx,
                                                       Selection.Append)
                        else:
                            self.KeyHandler(key='Escape')

            elif self.mode == Selection.New:
                if actor == self.parent.raw_actor:
                    if self.selection == None:
                        self.selection = Selection(self, actor, idx,
                                                   Selection.New)
                    else:
                        self.selection = Selection(self, actor,
                                                   self.selection.point.idx,
                                                   Selection.New, idx)
                        pts = self.selection.new()
                        change = InsertChange(self.selection, pts,
                                              self.selection.point.lane)
                        self.undoer.addChange(change)

                        self.mode = Selection.Append
                        end_pt = self.selection.end_point
                        self.selection = Selection(self, end_pt.actor,
                                                   end_pt.idx, Selection.Append)

    def LeftButtonReleaseEvent(self, obj, event):
        if self.moving and self.mode == 'edit':
            end_pos = self.selection.getPosition()
            vector = end_pos - self.selection.point.start_pos

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

            new_pos = np.array(new_pick_point[:-1])
            new_pos[2] = old_pos[2]

            # Snapping
            snapped = False
            (d, i) = self.parent.raw_kdtree2d.query(new_pos[:-1])
            if d < 1:
                new_pos = self.parent.raw_cloud.xyz[i]
                snapped = True

            old_pos = np.array(old_pos)
            change = new_pos - old_pos

            # Move the point
            self.selection.move(change)
            if snapped:
                self.selection.lowlight()
            else:
                self.selection.highlight()

            self.Render()

        x, y = self.iren.GetEventPosition()
        self.picker.Pick(x, y, 0, self.ren)
        idx = self.picker.GetPointId()
        actor = self.picker.GetActor()
        if idx >= 0 and self.selection and self.mode == Selection.Copy:
            # If we are in copy mode and we have selected a lane point
            if self.selection.copy_ready and actor == self.parent.raw_actor:
                if not self.selection.copying:
                    # Don't remove the lane the first time
                    self.selection.lowlight()
                    self.selection.copying = True
                else:
                    self.parent.removeLane(self.selection.point.lane)
                pts = self.selection.copy(idx)
        if actor in self.parent.lane_actors:
            self.hover_lane = self.parent.lane_actors.index(actor)
            self.hover_point = idx

    def lowlightAll(self):
        for i in self.highlighted_lanes:
            self.lowlightLane(i)
        self.highlighted_lanes = []

    def lowlightLane(self, num):
        actor = self.parent.lane_actors[num]
        lane = Selection(self, actor, 0, Selection.All)
        lane.lowlight()
        self.Render()

    def togglePick(self, lane=True):
        self.parent.raw_actor.SetPickable(not lane)
        for l in self.parent.lane_actors:
            l.SetPickable(lane)

    def listLaneModes(self):
        return [str(i) for i in xrange(self.parent.num_lanes)]

    def KeyHandler(self, obj=None, event=None, key=None):
        # Symbol names are declared in
        # GUISupport/Qt/QVTKInteractorAdapter.cxx
        # https://github.com/Kitware/VTK/
        if key == None:
            key = self.iren.GetKeySym()

        if key == 'q':
            self.KeyHandler(key='s')
            # TODO: confirm quit
            self.iren.TerminateApp()
            return

        elif key == 's':
            folder = sys.argv[1] + '/corrected_lanes/'
            try:
                os.mkdir(folder)
            except OSError:
                pass
            file_name = str(int(time.time()) / 10) + '0.npz'
            print 'Saved', folder + file_name
            file_name = folder + file_name
            self.parent.exportData(file_name)

        if not self.moving:
            if key == 'Escape':
                if self.selection != None:
                    if self.selection.copying:
                        return
                    if not self.selection.deleted:
                        self.selection.lowlight()
                    self.selection = None
                self.togglePick(lane=True)
                self.mode = 'edit'
                self.lowlightAll()

            elif key == 'bracketright':
                self.parent.lane_size = self.parent.lane_size + 0.5
                for actor in self.parent.lane_actors:
                    actor.GetProperty().SetPointSize(self.parent.lane_size)
                self.mode = 'edit'
            elif key == 'bracketleft':
                self.parent.lane_size = max(self.parent.lane_size - 0.5, 1)
                for actor in self.parent.lane_actors:
                    actor.GetProperty().SetPointSize(self.parent.lane_size)
                self.mode = 'edit'

            elif key == 'braceright':
                op = min(self.parent.raw_actor.GetProperty().GetOpacity() +
                         0.05, 1)
                self.parent.raw_actor.GetProperty().SetOpacity(op)
                self.mode = 'edit'
            elif key == 'braceleft':
                # Decrease the number of points selected
                op = max(self.parent.raw_actor.GetProperty().GetOpacity() -
                         0.05, 0.05)
                self.parent.raw_actor.GetProperty().SetOpacity(op)
                self.mode = 'edit'

            elif key in self.listLaneModes():
                self.lowlightAll()
                if self.mode + key in self.listLaneModes():
                    self.mode += key
                else:
                    self.mode = key
                actor = self.parent.lane_actors[int(self.mode)]
                lane = Selection(self, actor, 0, Selection.All)
                self.highlighted_lanes.append(int(self.mode))
                lane.highlight()

            elif key == 'd':
                # Don't allow changes when copying due to temporary lanes
                if self.mode == Selection.Copy:
                    return
                if self.mode in self.listLaneModes():
                    self.lowlightAll()
                    actor = self.parent.lane_actors[int(self.mode)]
                    lane_selection = Selection(self, actor, 0, Selection.All)
                    lane_selection.highlight()
                    del_section, lane_num = lane_selection.delete()
                    change = DeleteChange(lane_selection, del_section,
                                          lane_num)
                    self.undoer.addChange(change)
                    self.KeyHandler(key='Escape')
                elif not self.mode in [Selection.Delete, Selection.Copy]:
                    self.mode = Selection.Delete
                elif self.selection != None and self.selection.isSelected():
                    del_selection, lane = self.selection.delete()
                    change = DeleteChange(self.selection, del_selection, lane)
                    self.undoer.addChange(change)
                    self.KeyHandler(key='Escape')

            elif key == 'i':
                # Don't allow changes when copying due to temporary lanes
                if self.mode == Selection.Copy:
                    return
                self.KeyHandler(key='Escape')
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
                elif key == 'n':
                    self.mode = Selection.New
                    self.togglePick(lane=False)
            elif self.mode == Selection.Copy:
                if key == 'c':
                    if self.selection:
                        self.selection.copy_ready = True
                        self.togglePick(lane=False)

            elif key == 'f':
                if self.selection and self.selection.isSelected():
                    self.selection.fixup()
                elif self.mode in self.listLaneModes():
                    print 'Fixing up lane', self.mode
                    num = int(self.mode)
                    actor = self.parent.lane_actors[num]
                    sel = Selection(self, actor, 0, Selection.All)
                    sel.fixup()
                    print 'Fixup finished'
                elif self.mode != Selection.Fixup:
                    self.mode = Selection.Fixup

            # elif key == 'F':
            #     if self.mode == 'edit':
            #         print 'Fixing up all lanes'
            #         self.parent.fixupAllLanes()
            #         print 'Fixup finished'

            elif key == 'space':
                self.parent.running = not self.parent.running
                print 'Running' if self.parent.running else 'Paused'

            elif key == 'z':
                print 'Undo'
                self.undoer.undo()
                self.KeyHandler(key='Escape')
                self.Render()

            elif key == 'y':
                print 'Redo'
                self.undoer.redo()
                self.KeyHandler(key='Escape')
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
                    if self.parent.mk2_t > 0:
                        self.parent.mk2_t -= self.parent.small_step
                        self.parent.manual_change = -1

            elif key == 'Up':
                if not self.parent.running:
                    if not self.parent.finished():
                        self.parent.mk2_t += self.parent.small_step
                        self.parent.manual_change = 1

    def updateModeText(self):
        frame_num = self.parent.mk2_t
        tot_num = self.parent.video_reader.total_frame_count
        mode = self.mode
        txt = 'Frame (%d/%d) | Lane (%d, %d)\n' % (frame_num, tot_num,
                                                   self.hover_lane,
                                                   self.hover_point)

        if mode == 'edit':
            txt += 'Click to move lane | (i) - insert mode | (d) - ' + \
                   'delete mode | (f) - fixup mode'
        elif mode in self.listLaneModes():
            txt += 'Lane %s - All points' % mode
        elif mode == Selection.Delete:
            if self.selection == None:
                txt += 'Click to start delete segment | (esc) - edit mode'
            elif not self.selection.isSelected():
                txt = txt + 'Select another point to create delete segment'
            else:
                txt += '(d) - delete selected segment | click - ' + \
                       'change segment | (esc) - start over'
        elif mode == 'insert':
            txt += '(a) - append | (f) - fork | (c) - copy | (j) - join | ' + \
                   '(n) - new point'
        elif mode == Selection.Append:
            if self.selection == None:
                txt += 'Appending (1/2). Select a lane. Append will add ' + \
                       'points to the end or beginning of a lane'
            else:
                txt += 'Appending (2/2). Select a ground point'
        elif mode == Selection.Fork:
            if self.selection == None:
                txt += 'Forking (1/2). Select a point. Fork will create a ' + \
                       'new lane from an existing point'
            else:
                txt += 'Forking (2/2). Select a ground point'
        elif mode == Selection.Join:
            if self.selection == None:
                txt += 'Joining (1/2). Select a lane. Join will join two ' + \
                       'independent lanes into one'
            else:
                txt += 'Joining (2/2). Select a lane'
        elif mode == Selection.Copy:
            if self.selection == None:
                txt += 'Copy (1/3). Select a lane point to start copy.'
            elif not self.selection.copy_ready:
                txt += 'Copy (2/3). (c)- confirm selection | click - ' + \
                       'change segment | (esc) - start over'
            else:
                txt += 'Copy (3/3). Select a ground point to begin copy'
        elif mode == Selection.New:
            if self.selection == None:
                txt += 'New Point (1/2). Select a ground point to start a new lane'
            else:
                txt += 'New Point (2/2). Select a ground point to end a new lane'
        elif mode == Selection.Fixup:
            if self.selection == None:
                txt += 'Click to start fixup segment | (esc) - edit mode'
            elif not self.selection.isSelected():
                txt = txt + 'Select another point to create fixup segment'
            else:
                txt += '(f) - fixup selected segment | click - ' + \
                       'change segment | (esc) - start over'

        self.parent.text_info_actor.SetInput(txt)
        self.parent.text_info_actor.Modified()
        self.parent.text_shadow_actor.SetInput(txt)
        self.parent.text_shadow_actor.Modified()

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
        if len(sys.argv) <= 2 or '-h' in sys.argv or '--help' in sys.argv:
            print """Usage:
            LaneCorrector.py folder/ video.avi
            LaneCorrector.py folder/ video.avi lane_points.npz
            LaneCorrector.py folder/ video.avi background.npz lane_points.npz"""
            sys.exit(-1)
        args = parse_args(sys.argv[1], sys.argv[2])
        bg_file = glob.glob(args['fullname'] + '*bg.npz')[0]
        if len(sys.argv) == 4:
            sys.argv.insert(-1, bg_file)
        else:
            if len(sys.argv) == 3:
                sys.argv.append(bg_file)
            if len(sys.argv) <= 4:
                sys.argv.append(sys.argv[1] + '/multilane_points.npz')

        print sys.argv

        self.small_step = 10
        self.large_step = 50
        self.startup_complete = False

        ##### Grab all the transforms ######
        (self.imu_transforms_mk1,
         self.gps_data_mk1,
         self.gps_times_mk1) = get_transforms(args, 'mark1')

        (self.imu_transforms_mk2,
         self.gps_data_mk2,
         self.gps_times_mk2) = get_transforms(args, 'mark2')

        self.mk2_t = 0
        self.t = self.mk2_to_mk1(self.mk2_t)

        self.cur_imu_transform = self.imu_transforms_mk1[self.t, :,:]
        self.imu_kdtree = cKDTree(self.imu_transforms_mk1[:, :3, 3])

        self.params = args['params']
        self.lidar_params = self.params['lidar']
        self.T_from_i_to_l = np.linalg.inv(self.lidar_params['T_from_l_to_i'])
        cam_num = args['cam_num']
        self.cam_params = self.params['cam'][cam_num - 1]

        # Store the images
        gmap_step = 100
        pool = multiprocessing.Pool(processes=50)
        latlons = [tuple(row) for i, row in
                   enumerate(self.gps_data_mk1[::gmap_step, 1:3])]
        frames = [x * gmap_step for x in xrange(len(latlons))]
        pool.map(load_gmaps, zip(frames, latlons))
        pool.terminate()
        self.gmap = None

        # Whether to write video
        self.record = False
        # Is the flyover running
        self.running = True
        # Has the user changed the time
        self.manual_change = 0

        ###### Set up the renderers ######
        self.cloud_ren = vtk.vtkRenderer()
        self.cloud_ren.SetViewport(0, 0, 0.7, 1.0)
        self.cloud_ren.SetBackground(0, 0, 0)

        self.img_ren = vtk.vtkRenderer()
        self.img_ren.SetViewport(0.7, 0.0, 1.0, 0.37)
        # self.img_ren.SetInteractive(False)
        self.img_ren.SetBackground(0.1, 0.1, 0.1)

        self.gmap_ren = vtk.vtkRenderer()
        self.gmap_ren.SetViewport(0.75, 0.4, 1.0, 0.65)
        # self.gmap_ren.SetInteractive(False)
        self.gmap_ren.SetBackground(0.1, 0.1, 0.1)

        self.win = vtk.vtkRenderWindow()
        self.win.StereoCapableWindowOff()
        self.win.AddRenderer(self.cloud_ren)
        self.win.AddRenderer(self.img_ren)
        self.win.AddRenderer(self.gmap_ren)
        self.win.SetSize(800, 400)

        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren.SetRenderWindow(self.win)

        ###### Cloud Actors ######
        self.gmap_actor = None

        print 'Adding raw points'
        raw_npz = np.load(sys.argv[3])
        pts = raw_npz['data']

        self.raw_cloud = VtkPointCloud(pts[:, :3],
                                       np.ones(pts[:, :3].shape) * 255)
        self.raw_actor = self.raw_cloud.get_vtk_color_cloud()
        self.raw_actor.GetProperty().SetPointSize(5)
        self.raw_actor.GetProperty().SetOpacity(0.3)
        self.raw_actor.SetPickable(0)
        self.cloud_ren.AddActor(self.raw_actor)

        self.raw_kdtree = cKDTree(self.raw_cloud.xyz)
        self.raw_kdtree2d = cKDTree(self.raw_cloud.xyz[:, :-1])

        print 'Loading interpolated lanes'
        npz = np.load(sys.argv[4])
        init_num_lanes = int(npz['num_lanes'])
        if 'saved_t' in npz:
            self.init_t = int(npz['saved_t'])
        else:
            self.init_t = 0

        self.num_lanes = 0
        self.num_colors = 15
        self.colors = [hsv_to_rgb(float(i) / self.num_colors, .9, 1.) for i
                       in xrange(self.num_colors)]
        # The last color is for highlighting
        self.colors.append([1, 1, .4])
        self.colors = 255 * np.array(self.colors)

        self.lane_size = 3

        self.lane_clouds = []
        self.lane_actors = []
        self.lane_kdtrees = []

        for i in xrange(init_num_lanes):
            interp_lane = npz['lane' + str(i)]
            self.addLane(interp_lane)

        print 'Adding car'
        self.car = load_ply('../mapping/viz/gtr.ply')
        self.car.SetPickable(0)
        self.car.GetProperty().LightingOff()
        self.cloud_ren.AddActor(self.car)

        # Use our custom mouse interactor
        self.interactor = LaneInteractorStyle(self.iren, self.cloud_ren, self)
        self.iren.SetInteractorStyle(self.interactor)

        # Tell the user which mode we are in
        text_shadow = VtkText('Starting...', (8, 8))
        self.text_shadow_actor = text_shadow.get_vtk_text()
        text_info = VtkText('Starting...', (10, 10))
        self.text_info_actor = text_info.get_vtk_text()

        for actor in [self.text_shadow_actor, self.text_info_actor]:
            if actor == self.text_shadow_actor:
                actor.GetTextProperty().SetColor((0, 0, 0))
            actor.GetTextProperty().BoldOn()
            actor.GetTextProperty().SetFontSize(24)
            self.cloud_ren.AddActor(actor)

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

    def mk2_to_mk1(self, mk2_idx):
        t = self.gps_times_mk2[mk2_idx]
        mk1_idx = bisect.bisect(self.gps_times_mk1, t) - 1
        return mk1_idx

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

        cloud = VtkPointCloud(data[:, :3], np.ones((num_pts, 3)) *
                              (self.colors[lane_num % self.num_colors]))
        actor = cloud.get_vtk_color_cloud()

        actor.GetProperty().SetPointSize(self.lane_size)

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

    def refreshLaneColors(self):
        for actor in self.lane_actors:
            sel = Selection(self.interactor, actor, 0, Selection.All)
            sel.refreshColor()

    def removeLane(self, lane):
        actor = self.lane_actors[lane]
        self.cloud_ren.RemoveActor(actor)
        del self.lane_actors[lane]
        del self.lane_clouds[lane]
        del self.lane_kdtrees[lane]
        self.num_lanes -= 1

    def fixupAllLanes(self):
        for l in xrange(self.num_lanes):
            sel = Selection(self.interactor, self.lane_actors[l], 0,
                            Selection.All)
            sel.highlight()
            sel.fixup()
            sel.lowlight()

    def getCameraPosition(self, t, focus=100):
        offset = np.array([-75.0, 0, 25.0]) / 4
        # Rotate the camera so it is behind the car
        position = np.dot(self.imu_transforms_mk1[t, 0:3, 0:3], offset)
        position += self.imu_transforms_mk1[t, 0:3, 3] + position

        # Focus 10 frames in front of the car
        focal_point = self.imu_transforms_mk1[t + focus, 0:3, 3]
        return position, focal_point

    def exportData(self, file_name):
        lanes = {}
        lanes['num_lanes'] = self.num_lanes
        lanes['saved_t'] = self.mk2_t
        for num in xrange(self.num_lanes):
            lane = self.lane_clouds[num].xyz[:, :3]
            lanes['lane' + str(num)] = lane

        np.savez(file_name, **lanes)

    def finished(self, focus=100):
        return self.mk2_t + 2 * focus > self.video_reader.total_frame_count

    def update(self, iren, event):
        # Transform the car
        cloud_cam = self.cloud_ren.GetActiveCamera()

        # If we have gone backwards in time we need use setframe (slow)
        if self.manual_change != 0:
            self.video_reader.setFrame(self.mk2_t - 1)

        while self.video_reader.framenum <= self.mk2_t:
            (success, self.I) = self.video_reader.getNextFrame()

        # Copy the image so we can project points onto it
        I = self.I.copy()
        I = self.projectPointsOnImg(I)
        vtkimg = VtkImage(I)

        self.img_ren.RemoveActor(self.img_actor)
        self.img_actor = vtkimg.get_vtk_image()
        self.img_ren.AddActor(self.img_actor)

        if self.gmap != self.get_gmap():
            self.gmap = self.get_gmap()
            gmap_img = cv2.imread(self.gmap)
            gmap_vtk = VtkImage(gmap_img)
            self.gmap_ren.RemoveActor(self.gmap_actor)
            self.gmap_actor = gmap_vtk.get_vtk_image()
            center = (200, 200, 0)
            self.gmap_actor.SetOrigin(center)
            self.gmap_actor.RotateZ(self.gps_data_mk1[self.t, 9] + 90)
            self.gmap_ren.AddActor(self.gmap_actor)

        if self.running or self.manual_change:
            # Set camera position to in front of the car
            position, focal_point = self.getCameraPosition(self.t)
            cloud_cam.SetPosition(position)
            cloud_cam.SetFocalPoint(focal_point)

            # Update the car position
            self.cur_imu_transform = self.imu_transforms_mk1[self.t, :,:]
            transform = vtk_transform_from_np(self.cur_imu_transform)
            transform.RotateZ(90)
            transform.Translate(-2, -3, -2)
            self.car.SetUserTransform(transform)

            # If the user caused a manual change, reset it
            self.manual_change = 0

        # Initialization
        if not self.startup_complete:
            cloud_cam.SetViewUp(0, 0, 1)
            self.img_ren.ResetCamera()
            # These units are pixels
            self.img_ren.GetActiveCamera().SetClippingRange(100, 100000)
            self.img_ren.GetActiveCamera().Dolly(1.75)

            self.gmap_ren.ResetCamera()
            self.gmap_ren.GetActiveCamera().SetClippingRange(100, 100000)
            self.gmap_ren.GetActiveCamera().Dolly(1.75)

            self.mk2_t = self.init_t
            self.t = self.mk2_to_mk1(self.mk2_t)

            self.startup_complete = True
            self.manual_change = -1

        # Update the little text in the bottom left
        self.interactor.updateModeText()

        if self.record:
            self.interactor.writeVideo()

        if self.running:
            self.mk2_t += self.large_step

        if self.finished():
            self.mk2_t -= self.large_step
            if self.running == True:
                self.interactor.KeyHandler(key='space')

        self.t = self.mk2_to_mk1(self.mk2_t)

        iren.GetRenderWindow().Render()



if __name__ == '__main__':
    Blockworld()
