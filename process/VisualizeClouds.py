#!/usr/bin/python
# -*- coding: utf-8 -*-

from Q50_config import *
import sys, os
from GPSReader import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
import numpy as np
import cv2
from ArgParser import *
import bisect
from LidarTransforms import interp_transforms, transform_points_by_times
from VtkRenderer import *
import sys
import os
import time

global fnum
fnum = 0
START_LIDAR = 20

def cloudToPixels(cam, pts_wrt_cam): 
    width = 4
    (pix, J)  = cv2.projectPoints(pts_wrt_cam.transpose(), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), cam['KK'], cam['distort'])
    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] < 1039 - width/2)
    mask = np.logical_and(mask, pix[0,:] < 2079 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

def lidarPtsToPixels(pts_wrt_lidar_t, cam):
    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = dot(R_to_c_from_l(cam),
            pts_wrt_camera_t.transpose())
    pts_wrt_camera_t = np.vstack((pts_wrt_camera_t,
        np.ones((1, pts_wrt_camera_t.shape[1]))))
    pts_wrt_camera_t = dot(cam['E'], pts_wrt_camera_t)
    pts_wrt_camera_t = pts_wrt_camera_t[0:3,:]

    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)
    return (pix, mask)

class VideoCamera:
    def __init__(self, video_file, cam_params):
        vfilePath, vFile = os.path.split(video_file)
        self.cam_num = int(vFile[-5])
        self.video_reader = VideoReader(video_file)
        self.cam = cam_params

class LDRGrabberCallback:

    def __init__(self):
        self.pointCloud = [ ]

    def execute(self, obj, event):
        global fnum

        images = [ ]
        for c in cameras:
            for _ in range(5):
                (success, I) = c.video_reader.getNextFrame()
            images.append(I.copy())

        fnum = cameras[0].video_reader.framenum * 2
        if fnum < START_LIDAR:
            return

        t = gps_times_mark2[fnum]
        data, t_data = lidar_loader.loadLDRWindow(t-50000, 0.1)


        pts = data[:, 0:3].transpose()
        pts = np.vstack((pts, np.ones((1, pts.shape[1]))))
        pts = np.dot(T_from_l_to_i, pts)
        # Shift points according to timestamps instead of using transform of full sweep
        transform_points_by_times(pts, t_data, imu_transforms_mark2, gps_times_mark2)
        pts_wrt_imu_0 = pts.copy() 
        pts_wrt_imu_t = np.dot(np.linalg.inv(imu_transforms_mark2[fnum,:,:]), pts)

        # transform points from imu_t to lidar_t
        pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t)
        color_data = np.zeros((data.shape[0], 3))

        for idx in range(len(cameras)):
            I = images[idx]
            (pix, mask) = lidarPtsToPixels(pts_wrt_lidar_t, cameras[idx].cam)
            intensity = data[mask, 3]
            heat_colors = heatColorMapFast(intensity, 0, 100)
            I_color_data = np.zeros((np.count_nonzero(mask), 3))
            I_color_data[:,0] = I[pix[1,mask], pix[0,mask], 2]
            I_color_data[:,1] = I[pix[1,mask], pix[0,mask], 1]
            I_color_data[:,2] = I[pix[1,mask], pix[0,mask], 0]

            color_data[mask, :] = I_color_data
            for p in range(4):
                I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
                I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]
            cv2.imshow("video"+str(idx), cv2.pyrDown(I))

        #pts = pts_wrt_lidar_t.transpose()
        #print pts
        pts = pts_wrt_imu_0.transpose()

        #self.pointCloud = VtkPointCloud(pts[:, 0:3], data[:, 3])
        #actor = self.pointCloud.get_vtk_cloud(zMin=0, zMax=255)
        self.pointCloud = VtkPointCloud(pts_wrt_lidar_t.transpose()[:,0:3], color_data)
        actor = self.pointCloud.get_vtk_color_cloud()
        #self.pointCloud.append(VtkPointCloud(pts[:,0:3], color_data))
        #actor = self.pointCloud[-1].get_vtk_color_cloud()
        iren = obj

        if fnum > START_LIDAR:
            iren.GetRenderWindow().GetRenderers().GetFirstRenderer().RemoveActor(self.actor)
        self.actor = actor
        iren.GetRenderWindow().GetRenderers().GetFirstRenderer().AddActor(self.actor)
        if fnum == START_LIDAR:
            iren.GetRenderWindow().GetRenderers().GetFirstRenderer().ResetCamera()

        iren.GetRenderWindow().Render()

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    params = args['params']
    gps_reader_mark2 = GPSReader(args['gps_mark2'])
    gps_data_mark2 = gps_reader_mark2.getNumericData()

    lidar_loader = LDRLoader(args['frames'])
    imu_transforms_mark2 = IMUTransforms(gps_data_mark2)
    gps_times_mark2 = utc_from_gps_log_all(gps_data_mark2)

    T_from_l_to_i = params['lidar']['T_from_l_to_i']
    T_from_i_to_l = np.linalg.inv(T_from_l_to_i)

    cameras = [ ] 
    for cam_num in range(3,7):
        vfilePath, vFile = os.path.split(args['video'])
        vFile = vFile[:-5] + str(cam_num) + vFile[-4:]
        video_file = os.path.join(vfilePath, vFile)
        camera = VideoCamera(video_file, params['cam'][cam_num-1])
        cameras.append(camera)

    # Render Window
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0., 0., 0.)
    renderer.ResetCamera()
    
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(1280, 720)

    # Interactor

    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    renderWindowInteractor.SetInteractorStyle(mouseInteractor)
    renderWindow.Render()

    cb = LDRGrabberCallback()
    renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
    timerId = renderWindowInteractor.CreateRepeatingTimer(10)
    renderWindowInteractor.Start()
