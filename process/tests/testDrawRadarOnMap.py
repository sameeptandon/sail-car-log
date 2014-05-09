#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys, os
from VtkRenderer import *
import numpy as np
from RadarTransforms import *
from LidarTransforms import *
from Q50_config import *
from ArgParser import *
from GPSReader import *
from GPSTransforms import IMUTransforms
from transformations import euler_from_matrix
import math

class ImageGrabberCallback:

    def __init__(self, args, npz_file):
        self.map_file = args['map']
        self.lidar_params = args['params']['lidar']
        self.radar_params = args['params']['radar']

        self.radar_actors = []
        
        self.rdr_pts = loadRDRCamMap(self.map_file)
        self.count = 0

        self.lidar_actor = None
        self.lidar_data = np.load(npz_file)['data']
        
        gps_filename = args['gps']
        gps_reader = GPSReader(gps_filename)
        GPSData = gps_reader.getNumericData()
        self.imu_transforms = IMUTransforms(GPSData)

    def execute(self, iren, event):
        fren = iren.GetRenderWindow().GetRenderers().GetFirstRenderer()

        radar_data = loadRDR(self.rdr_pts[self.count])[0]
        if radar_data.shape[0] > 0:
            #Convert from radar to lidar ref-frame
            radar_data[:, :3] = calibrateRadarPts(radar_data[:, :3], self.radar_params)

            #Convert from lidar to IMU ref-frame
            radar_data[:, :3] = np.dot(self.lidar_params['T_from_l_to_i'][:3, :3], \
                radar_data[:, :3].transpose()).transpose()

            h_radar_data = np.hstack((radar_data[:, :3], np.ones((radar_data.shape[0], 1))))

            radar_data[:, :3] = np.dot(self.imu_transforms[self.count], \
                h_radar_data.transpose()).transpose()[:, :3]
            
            #Insert the Q50 position at the beginning
            me_l = 4.1
            me_w = 2.1
            me_x = self.imu_transforms[self.count, 0, 3]-me_l/2.
            me_y = self.imu_transforms[self.count, 1, 3]
            me_z = self.imu_transforms[self.count, 2, 3]
            me = [me_x, me_y, me_z, me_l, me_w, 0, 0, 0]
            radar_data = np.vstack((np.array(me), radar_data))

            for i in xrange(len(self.radar_actors)):
                fren.RemoveActor(self.radar_actors[i])

            self.radar_actors = []
            self.radar_clouds = []

            for i in xrange(radar_data.shape[0]):
                self.radar_clouds.append(VtkBoundingBox(radar_data[i, :]))
                (ax, ay, az) = euler_from_matrix(self.imu_transforms[self.count])
                box = self.radar_clouds[i].get_vtk_box(rot = az*180/math.pi)
                if i == 0:
                    box.GetProperty().SetColor((1, 0, 0))
                self.radar_actors.append(box)
                fren.AddActor(self.radar_actors[i])

            if self.count == 0:
                self.lidar_cloud = VtkPointCloud(self.lidar_data[:, :3], self.lidar_data[:,3])
                self.lidar_actor = self.lidar_cloud.get_vtk_cloud(zMin=0, zMax=255)
                fren.AddActor(self.lidar_actor)
                self.gpsPointCloud = VtkPointCloud(self.imu_transforms[:, 0:3, 3], 
                                                np.zeros(self.imu_transforms.shape[0]))
                fren.AddActor(self.gpsPointCloud.get_vtk_cloud())

            # if self.count == 0:
                # fren.ResetCamera()
                # fren.GetActiveCamera().Zoom(1.6)
            cam_center = [x for x in self.radar_actors[0].GetCenter()]
            cam_center[2] += 150
            fren.GetActiveCamera().SetPosition(cam_center)
            fren.GetActiveCamera().SetFocalPoint(self.radar_actors[0].GetCenter())

        self.count += 5
        iren.GetRenderWindow().Render()

if __name__ == '__main__': 
    args = parse_args(sys.argv[1], sys.argv[2])

    renderWindow = vtk.vtkRenderWindow()
    renderWindow.SetSize(1280/2, 960/2)

    renderer = vtk.vtkRenderer()
    renderWindow.AddRenderer(renderer)

    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    renderWindowInteractor.SetInteractorStyle(mouseInteractor)
    renderWindow.Render()
    cb = ImageGrabberCallback(args, sys.argv[3])
    
    renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
    timerId = renderWindowInteractor.CreateRepeatingTimer(1)
    renderWindowInteractor.Start()