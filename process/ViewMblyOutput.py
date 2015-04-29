#!/usr/bin/python
# -*- coding: utf-8 -*-
# Usage: LaneCorrector.py folder/ cam.avi raw_lidar.npz multilane_points.npz

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
from GPSTransforms import IMUTransforms, absoluteTransforms
from LaneMarkingHelper import BackProjector, DataTree, get_transforms, mk2_to_mk1, projectPointsOnImg, VTKCloudTree, VTKPlaneTree
from LidarTransforms import R_to_c_from_l, utc_from_gps_log_all
from MblyTransforms import MblyLoader, projectPoints, calibrateMblyPts
from VideoReader import VideoReader
from VtkRenderer import VtkPointCloud, VtkText, VtkImage, VtkPlane, VtkLine, VtkBoundingBox
from mbly_obj_pb2 import Object
from transformations import euler_from_matrix, euler_matrix

def vtk_transform_from_np(np4x4):
    vtk_matrix = vtk.vtkMatrix4x4()
    for r in range(4):
        for c in range(4):
            vtk_matrix.SetElement(r, c, np4x4[r, c])
    transform = vtk.vtkTransform()
    transform.SetMatrix(vtk_matrix)
    return transform

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

class LaneInteractorStyle (vtk.vtkInteractorStyleTrackballCamera):

    def __init__(self, iren, ren, parent):
        self.iren = iren
        self.ren = ren
        self.parent = parent

        self.moving = False

        self.SetMotionFactor(10.0)

        self.AutoAdjustCameraClippingRangeOff()
        self.ren.GetActiveCamera().SetClippingRange(0.01, 500)

        self.AddObserver('MouseWheelForwardEvent', self.MouseWheelForwardEvent)
        self.AddObserver('MouseWheelBackwardEvent', self.MouseWheelBackwardEvent)

        self.AddObserver('LeftButtonPressEvent', self.LeftButtonPressEvent)
        self.AddObserver('LeftButtonReleaseEvent', self.LeftButtonReleaseEvent)

        # Add keypress event
        self.AddObserver('CharEvent', self.KeyHandler)

    def MouseWheelForwardEvent(self, obj, event):
        self.OnMouseWheelForward()

    def MouseWheelBackwardEvent(self, obj, event):
        self.OnMouseWheelBackward()

    def LeftButtonPressEvent(self, obj, event):
        self.OnLeftButtonDown()

    def LeftButtonReleaseEvent(self, obj, event):
        self.OnLeftButtonUp()

    def KeyHandler(self, obj=None, event=None, key=None):
        # Symbol names are declared in
        # GUISupport/Qt/QVTKInteractorAdapter.cxx
        # https://github.com/Kitware/VTK/
        if key == None:
            key = self.iren.GetKeySym()

        if key == 'q':
            self.iren.TerminateApp()
            return
        elif key == 'space':
            self.parent.running = not self.parent.running
            print 'Running' if self.parent.running else 'Paused'

        elif key == 'Down':
            if not self.parent.running:
                if self.parent.mk2_t > 0:
                    self.parent.mk2_t -= self.parent.small_step
                    self.parent.t = self.parent.mk2_to_mk1()
                    self.parent.manual_change = -1

        elif key == 'Up':
            if not self.parent.running:
                if not self.parent.finished():
                    self.parent.mk2_t += self.parent.small_step
                    self.parent.t = self.parent.mk2_to_mk1()
                    self.parent.manual_change = 1

        elif key == 'o':
            self.parent.mbly_rot[0] -= 0.001
        elif key == 'u':
            self.parent.mbly_rot[0] += 0.001
        elif key == 'i':
            self.parent.mbly_rot[1] -= 0.001
        elif key == 'k':
            self.parent.mbly_rot[1] += 0.001
        elif key == 'l':
            self.parent.mbly_rot[2] -= 0.001
        elif key == 'j':
            self.parent.mbly_rot[2] += 0.001

        elif key == 'plus':
            self.parent.mbly_T[0] += 0.1
        elif key == 'minus':
            self.parent.mbly_T[0] -= 0.1
        elif key == 'd':
            self.parent.mbly_T[1] -= 0.1
        elif key == 'a':
            self.parent.mbly_T[1] += 0.1
        elif key == 'w':
            self.parent.mbly_T[2] -= 0.1
        elif key == 's':
            self.parent.mbly_T[2] += 0.1

        print 'R:', self.parent.mbly_rot
        print 'T:', self.parent.mbly_T


    def Render(self):
        self.iren.GetRenderWindow().Render()


class Blockworld:

    def __init__(self):
        if len(sys.argv) <= 2 or '--help' in sys.argv:
            print """Usage:
            {name} folder/ video.avi
            """.format(name = sys.argv[0])
            sys.exit(-1)
        args = parse_args(sys.argv[1], sys.argv[2])
        self.args = args

        bg_file = glob.glob(args['fullname'] + '*bg.npz')[0]
        print sys.argv

        self.small_step = 5
        self.large_step = 10
        self.startup_complete = False

        ##### Grab all the transforms ######
        self.absolute = False
        (self.imu_transforms_mk1,
         self.gps_data_mk1,
         self.gps_times_mk1) = get_transforms(args, 'mark1', self.absolute)

        (self.imu_transforms_mk2,
         self.gps_data_mk2,
         self.gps_times_mk2) = get_transforms(args, 'mark2', self.absolute)

        self.mk2_t = 0
        self.t = self.mk2_to_mk1()

        self.cur_imu_transform = self.imu_transforms_mk1[self.t, :,:]
        self.imu_kdtree = cKDTree(self.imu_transforms_mk1[:, :3, 3])

        self.params = args['params']
        self.lidar_params = self.params['lidar']
        self.T_from_i_to_l = np.linalg.inv(self.lidar_params['T_from_l_to_i'])
        cam_num = args['cam_num']
        self.cam_params = self.params['cam'][cam_num]

        # Load the MobilEye file
        self.mbly_loader = MblyLoader(args)
        # self.mbly_rot = [0.007, -0.01, -0.02]
        self.mbly_rot = [0.0, 0.0, 0.0]
        self.mbly_T = [0.0, 0.0, -2.0]
        # self.mbly_T = [0.762, 0.381, -0.9252]

        # Is the flyover running
        self.running = False
        # Has the user changed the time
        self.manual_change = 0

        ###### Set up the renderers ######
        self.cloud_ren = vtk.vtkRenderer()
        self.cloud_ren.SetViewport(0, 0, 0.7, 1.0)
        self.cloud_ren.SetBackground(0, 0, 0)

        self.img_ren = vtk.vtkRenderer()
        # self.img_ren.SetViewport(0.7, 0.0, 1.0, 0.37)
        self.img_ren.SetViewport(0.5, 0.0, 1.0, 0.5)
        self.img_ren.SetInteractive(False)
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
        raw_npz = np.load(bg_file)
        pts = raw_npz['data']

        raw_cloud = VtkPointCloud(pts[:, :3], np.ones(pts[:, :3].shape) * 255)
        raw_actor = raw_cloud.get_vtk_color_cloud()

        self.raw_lidar = VTKCloudTree(raw_cloud, raw_actor, build_tree=False)
        self.raw_lidar_2d = DataTree(self.raw_lidar.pts[:, :-1], build_tree=False)

        self.raw_lidar.actor.GetProperty().SetPointSize(2)
        self.raw_lidar.actor.GetProperty().SetOpacity(0.1)
        self.raw_lidar.actor.SetPickable(0)
        self.cloud_ren.AddActor(self.raw_lidar.actor)

        print 'Adding car'
        self.car = load_ply('../mapping/viz/gtr.ply')
        self.car.SetPickable(0)
        self.car.GetProperty().LightingOff()
        self.cloud_ren.AddActor(self.car)

        self.mbly_vtk_boxes = []
        # Car: 0, Truck: 1, Bike: 2, Other: 3-7
        red = np.array((1, 0, 0))
        green = np.array((0, 1, 0))
        blue = np.array((0, 0, 1))
        white = red+green+blue
        self.mbly_obj_colors = [red, green, blue, white]

        self.mbly_vtk_lanes = []
        # Dashed: 0, Solid: 1, undecided: 2, Edge: 3, Double: 4, Botts_Dots: 5
        self.mbly_lane_color = [green, green, red, blue, white, green]
        self.mbly_lane_size = [2, 3, 1, 1, 2, 1]
        self.mbly_lane_subsamp = [20, 1, 1, 1, 1, 40]

        # Use our custom mouse interactor
        self.interactor = LaneInteractorStyle(self.iren, self.cloud_ren, self)
        self.iren.SetInteractorStyle(self.interactor)

        ###### 2D Projection Actors ######
        self.video_reader = VideoReader(args['video'])
        self.img_actor = None

        self.I = None
        ###### Add Callbacks ######
        print 'Rendering'

        self.iren.Initialize()

        # Set up time
        self.iren.AddObserver('TimerEvent', self.update)
        self.timer = self.iren.CreateRepeatingTimer(10)

        self.iren.Start()

    def mk2_to_mk1(self, mk2_idx=-1):
        if mk2_idx == -1:
            mk2_idx = self.mk2_t
        return mk2_to_mk1(mk2_idx, self.gps_times_mk1, self.gps_times_mk2)

    def getCameraPosition(self, t, focus=100):
        offset = np.array([-75.0, 0, 25.0]) / 4
        # Rotate the camera so it is behind the car
        position = np.dot(self.imu_transforms_mk1[t, 0:3, 0:3], offset)
        position += self.imu_transforms_mk1[t, 0:3, 3] + position

        # Focus 10 frames in front of the car
        focal_point = self.imu_transforms_mk1[t + focus, 0:3, 3]
        return position, focal_point

    def finished(self, focus=100):
        return self.mk2_t + 2 * focus > self.video_reader.total_frame_count

    def update(self, iren, event):
        # Get the cameras
        cloud_cam = self.cloud_ren.GetActiveCamera()
        img_cam = self.img_ren.GetActiveCamera()

        # Initialization
        if not self.startup_complete:
            cloud_cam.SetViewUp(0, 0, 1)
            self.mk2_t = 0
            self.t = self.mk2_to_mk1()

            self.startup_complete = True
            self.manual_change = -1 # Force an update for the camera

        # Update the time (arrow keys also update time)
        if self.running:
            self.mk2_t += self.large_step
        if self.finished():
            self.mk2_t -= self.large_step
            if self.running == True:
                self.interactor.KeyHandler(key='space')

        # Get the correct gps time (mk2 is camera time)
        self.t = self.mk2_to_mk1()
        self.cur_imu_transform = self.imu_transforms_mk1[self.t, :, :]
        # Get the correct frame to show
        (success, self.I) = self.video_reader.getFrame(self.mk2_t)

        # Update the gps time
        self.cur_gps_time = self.gps_times_mk2[self.mk2_t]

        # Make sure the calibration has been updated
        self.mbly_R = euler_matrix(*self.mbly_rot)[:3, :3]

        if self.running or self.manual_change:
            # Set camera position to in front of the car
            position, focal_point = self.getCameraPosition(self.t)
            cloud_cam.SetPosition(position)
            cloud_cam.SetFocalPoint(focal_point)

            # Update the car position
            transform = vtk_transform_from_np(self.cur_imu_transform)
            transform.RotateZ(90)
            transform.Translate(-2, -3, -2)
            self.car.SetUserTransform(transform)

            # If the user caused a manual change, reset it
            self.manual_change = 0

        # Copy the image so we can project points onto it
        I = self.I.copy()

        # Add the lanes to the cloud
        mbly_lanes = self.mbly_loader.loadLane(self.cur_gps_time)
        lanes_wrt_mbly = self.mblyLaneAsNp(mbly_lanes)
        self.addLaneToCloud(lanes_wrt_mbly)
        # Add the lanes to the image copy
        I = self.addLaneToImg(I, lanes_wrt_mbly)

        # Add the objects (cars) to the cloud
        mbly_objs = self.mbly_loader.loadObj(self.cur_gps_time)
        objs_wrt_mbly = self.mblyObjAsNp(mbly_objs)
        self.addObjToCloud(objs_wrt_mbly)
        # Add the lanes to the image copy
        I = self.addObjToImg(I, objs_wrt_mbly)

        vtkimg = VtkImage(I)
        self.img_ren.RemoveActor(self.img_actor)
        self.img_actor = vtkimg.get_vtk_image()
        self.img_ren.AddActor(self.img_actor)

        # We need to draw the image before we run ResetCamera or else
        # the image is too small
        self.img_ren.ResetCamera()
        img_cam.SetClippingRange(100, 100000) # These units are pixels
        img_cam.Dolly(1.75)

        self.iren.GetRenderWindow().Render()

    def xformMblyToGlobal(self, pts_wrt_mbly):
        # TODO: Need to tranform from imu to gps frame of reference
        car_pos = self.imu_transforms_mk1[self.t, 0:3, 0:4]

        pts = pts_wrt_mbly
        # Puts points in lidar FOR
        pts_wrt_lidar = calibrateMblyPts(pts_wrt_mbly, self.mbly_T, \
                                         self.mbly_R[:3,:3])
        # Make points homogoneous
        hom_pts = np.hstack((pts_wrt_lidar[:, :3], np.ones((pts.shape[0], 1))))
        # Put in global FOR
        pts_wrt_world = np.dot(car_pos, hom_pts.T).T
        # Add metadata back to output
        pts_wrt_world = np.hstack((pts_wrt_world, pts[:, 3:]))
        return pts_wrt_world


    def mblyObjAsNp(self, mbly_objs):
        """Turns a mobileye object pb message into a numpy array with format:
        [x, y, 0, length, width, type]

        """
        pts_wrt_mbly = []
        for obj in mbly_objs:
            pt_wrt_mbly = [obj.pos_x, obj.pos_y, 0, obj.length, \
                           obj.width, obj.obj_type]
            pts_wrt_mbly.append(pt_wrt_mbly)
        return np.array(pts_wrt_mbly)

    def mblyLaneAsNp(self, mbly_lane):
        """Turns a mobileye lane into a numpy array with format:
        [C0, C1, C2, C3, lane_id, lane_type, view_range]

        Y = C3*X^3 + C2*X^2 + C1*X + C0.
        X is longitudinal distance from camera (positive right!)
        Y is lateral distance from camera

        lane_id is between -2 and 2, with -2 being the farthest left,
        and 2 being the farthest right lane. There is no 0 id.

        """
        lanes_wrt_mbly = []
        for l in mbly_lane:
            lane_wrt_mbly = [l.C0, l.C1, l.C2, l.C3, l.lane_id, l.lane_type, \
                             l.view_range]
            lanes_wrt_mbly.append(lane_wrt_mbly)
        return np.array(lanes_wrt_mbly)

    def addObjToCloud(self, objs_wrt_mbly):
        """ Add the mobileye returns to the 3d scene """
        mbly_vtk_boxes = []

        car_pos = self.imu_transforms_mk1[self.t, 0:3, 0:4]
        objs_wrt_world = self.xformMblyToGlobal(objs_wrt_mbly)

        # Draw each box
        car_rot = euler_from_matrix(car_pos[:,:3])[2] * 180 / math.pi
        for o in objs_wrt_world:
            properties = tuple(o)
            # Create the vtk object
            box = VtkBoundingBox(properties)
            actor = box.get_vtk_box(car_rot)
            color = self.mbly_obj_colors[int(o[5])]
            actor.GetProperty().SetColor(*color)
            mbly_vtk_boxes.append(box)

        # Remove old boxes
        for vtk_box in self.mbly_vtk_boxes:
            self.cloud_ren.RemoveActor(vtk_box.actor)
        # Update to new actors
        self.mbly_vtk_boxes = mbly_vtk_boxes
        # Draw new boxes
        for vtk_box in self.mbly_vtk_boxes:
            self.cloud_ren.AddActor(vtk_box.actor)

    def getLanePointsFromModel(self, lane_wrt_mbly):
        num_pts = 200

        # from model: Y = C3*X^3 + C2*X^2 + C1*X + C0.
        X = np.linspace(0, 80, num=num_pts)
        X = np.vstack((np.ones(X.shape), X, np.power(X, 2), np.power(X, 3)))
        # Mbly uses Y-right as positive, we use Y-left as positive
        Y = -1 * np.dot(lane_wrt_mbly[:4], X)
        lane_pts_wrt_mbly = np.vstack((X[1, :], Y, np.zeros((1, num_pts)))).T

        return lane_pts_wrt_mbly

    def addLaneToCloud(self, lane_wrt_mbly):

        for lane in self.mbly_vtk_lanes:
            self.cloud_ren.RemoveActor(lane.actor)
            self.mbly_vtk_lanes = []

        for i in xrange(lane_wrt_mbly.shape[0]):
            type = int(lane_wrt_mbly[i, 5])
            color = self.mbly_lane_color[type] * 255
            size = self.mbly_lane_size[type]
            subsamp = self.mbly_lane_subsamp[type]

            lane_pts_wrt_mbly = self.getLanePointsFromModel(lane_wrt_mbly[i, :])
            pts_wrt_global = self.xformMblyToGlobal(lane_pts_wrt_mbly)
            pts_wrt_global = pts_wrt_global[::subsamp]

            num_pts = pts_wrt_global.shape[0]
            vtk_lane = VtkPointCloud(pts_wrt_global, np.tile(color, (num_pts, 1)))
            actor = vtk_lane.get_vtk_color_cloud()
            actor.GetProperty().SetPointSize(size)

            self.mbly_vtk_lanes.append(vtk_lane)

            self.cloud_ren.AddActor(actor)


    def addObjToImg(self, I, objs_wrt_mbly):
        """Takes an image and the mbly objects. Converts the objects into corners of a
        bounding box and draws them on the image

        """
        if objs_wrt_mbly.shape[0] == 0:
            return None

        pix = []
        width = objs_wrt_mbly[:, 4]

        # Assuming the point in obs_wrt_mbly are the center of the object, draw
        # a box .5 m below, .5 m above, -.5*width left, .5*width right. Keep the
        # same z position
        for z in [-.5, .5]:
            for y in [-.5, .5]:
                offset = np.zeros((width.shape[0], 3))
                offset[:, 1] = width*y
                offset[:, 2] = z
                pt = objs_wrt_mbly[:, :3] + offset
                pix.append(projectPoints(pt, self.args, self.mbly_T, \
                                         self.mbly_R)[:, 3:])

        pix = np.array(pix, dtype=np.int32)
        pix = np.swapaxes(pix, 0, 1)

        # Draw a line between projected points
        for i, corner in enumerate(pix):
            # Get the color of the box and convert RGB -> BGR
            color = self.mbly_obj_colors[int(objs_wrt_mbly[i, 5])][::-1] * 255
            corner = tuple(map(tuple, corner))
            cv2.rectangle(I, corner[0], corner[3], color, 2)

        return I

    def addLaneToImg(self, I, lanes_wrt_mbly):
        """Takes an image and the 3d lane points. Projects these points onto the image
        """
        if lanes_wrt_mbly.shape[0] == 0:
            return None

        pix = []
        for i in xrange(len(self.mbly_vtk_lanes)):
            type = int(lanes_wrt_mbly[i, 5])
            color = self.mbly_lane_color[type] * 255
            size = self.mbly_lane_size[type]
            subsamp = self.mbly_lane_subsamp[type]

            pts = self.getLanePointsFromModel(lanes_wrt_mbly[i])[::subsamp]
            proj_pts = projectPoints(pts, self.args, self.mbly_T, self.mbly_R)
            proj_pts = proj_pts[:, 3:].astype(np.int32, copy = False)

            img_mask = (proj_pts[:, 0] < 1280) & (proj_pts[:, 0] >= 0) &\
                       (proj_pts[:, 1] < 800) & (proj_pts[:, 1] >= 0)

            proj_pts = proj_pts[img_mask]

            for pt_i in xrange(proj_pts.shape[0]):
                # cv2 only takes tuples at points
                pt = tuple(proj_pts[pt_i, :])
                # Make sure to convert to bgr
                cv2.circle(I, pt, size, color[::-1], thickness=-size)
        return I


if __name__ == '__main__':
    Blockworld()
