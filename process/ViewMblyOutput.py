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
from MblyTransforms import MblyLoader
from VideoReader import VideoReader
from VtkRenderer import VtkPointCloud, VtkText, VtkImage, VtkPlane, VtkLine, VtkBoundingBox
from mbly_obj_pb2 import Object
from transformations import euler_from_matrix

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


    def Render(self):
        self.iren.GetRenderWindow().Render()


class Blockworld:

    def __init__(self):
        if len(sys.argv) <= 2 or '-h' in sys.argv or '--help' in sys.argv:
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
        self.mbly_loader = MblyLoader(args['mbly_obj'])

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

        self.raw_lidar = VTKCloudTree(raw_cloud, raw_actor)
        self.raw_lidar_2d = DataTree(self.raw_lidar.pts[:, :-1])

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
        self.mbly_box_colors = [(1,0,0),(0,1,0),(0,0,1),(1,1,1)]
        # Use our custom mouse interactor
        self.interactor = LaneInteractorStyle(self.iren, self.cloud_ren, self)
        self.iren.SetInteractorStyle(self.interactor)

        ###### 2D Projection Actors ######
        self.video_reader = VideoReader(args['video'])
        self.img_actor = None

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
        # Transform the car
        cloud_cam = self.cloud_ren.GetActiveCamera()

        # If we have gone backwards in time we need use setframe (slow)
        # if self.manual_change != 0:
        self.video_reader.setFrame(self.mk2_t - 1)

        while self.video_reader.framenum <= self.mk2_t:
            (success, self.I) = self.video_reader.getNextFrame()


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

            gps_time = self.gps_times_mk2[self.mk2_t]
            mbly_objs = self.mbly_loader.loadMblyWindow(gps_time)
            xform = self.cur_imu_transform
            car_pos = self.imu_transforms_mk1[self.t, 0:3, 0:4]

            mbly_vtk_boxes = []
            # (x, y, z) = (0.762, 0.0381, 0.9652) meters to lidar
            for obj in mbly_objs:
                print obj
                X = np.array((obj.pos_x, obj.pos_y, 0, 1))
                # Add offsets
                pts_wrt_lidar = X + np.array((0.762, 0.0381, -0.9652, 0))

                # Move points to car ref frame
                X = np.dot(car_pos, pts_wrt_lidar)
                w = obj.width
                l = obj.length
                properties = (X[0], X[1], X[2], l, w)

                # Create the vtk object
                box = VtkBoundingBox(properties)
                rot = euler_from_matrix(car_pos[:,:3])[2] * 180 / math.pi
                actor = box.get_vtk_box(rot)
                color = self.mbly_box_colors[obj.obj_type]
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
                print vtk_box.corners

        # Copy the image so we can project points onto it
        I = self.I.copy()
        # I = self.projectPointsOnImg(I)
        vtkimg = VtkImage(I)

        self.img_ren.RemoveActor(self.img_actor)
        self.img_actor = vtkimg.get_vtk_image()
        self.img_ren.AddActor(self.img_actor)

        # Initialization
        if not self.startup_complete:
            cloud_cam.SetViewUp(0, 0, 1)
            self.img_ren.ResetCamera()
            # These units are pixels
            self.img_ren.GetActiveCamera().SetClippingRange(100, 100000)
            self.img_ren.GetActiveCamera().Dolly(1.75)

            self.mk2_t = 0
            self.t = self.mk2_to_mk1()

            self.startup_complete = True
            self.manual_change = -1

        if self.running:
            self.mk2_t += self.large_step

        if self.finished():
            self.mk2_t -= self.large_step
            if self.running == True:
                self.interactor.KeyHandler(key='space')

        self.t = self.mk2_to_mk1()

        self.iren.GetRenderWindow().Render()

    def projectPointsOnImg(self, I, show_lidar=False):
        car_pos = self.imu_transforms_mk1[self.t, 0:3, 3]

        if show_lidar:
            # Find the closest point
            lidar_tree = self.raw_lidar
            I = projectPointsOnImg(I, lidar_tree, self.imu_transforms_mk1,
                                   self.t, self.T_from_i_to_l, self.cam_params,
                                   [255, 255, 255])

        return I

if __name__ == '__main__':
    Blockworld()
