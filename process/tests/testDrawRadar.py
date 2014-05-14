#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys, os
from VtkRenderer import *
import numpy as np
from RadarTransforms import *
from LidarTransforms import *
from Q50_config import *


class ImageGrabberCallback:

    def __init__(self, map_file):
        self.map_file = map_file
        self.radar_params = LoadParameters('q50_4_3_14_params')['radar']

        self.lidar_actor = None

        self.radar_actors = []

        self.clouds = loadLDRCamMap(map_file)
        self.rdr_pts = loadRDRCamMap(map_file)
        self.count = 0

    def execute(self, iren, event):
        fren = iren.GetRenderWindow().GetRenderers().GetFirstRenderer()

        radar_data = loadRDR(self.rdr_pts[self.count])[0]
        radar_data[:, :3] = calibrateRadarPts(radar_data[:, :3], self.radar_params)
        if radar_data.shape[0] > 0:
            mask = (radar_data[:, 5] > 5)
            mask &= (radar_data[:, 6] > -20)
            radar_data = radar_data[mask]

        if radar_data.shape[0] > 0:
            for i in xrange(len(self.radar_actors)):
                fren.RemoveActor(self.radar_actors[i])

            self.radar_actors = []
            self.radar_clouds = []

            for i in xrange(radar_data.shape[0]):
                self.radar_clouds.append(VtkBoundingBox(radar_data[i, :]))

                self.radar_actors.append(self.radar_clouds[i].get_vtk_box())
                fren.AddActor(self.radar_actors[i])

            lidar_data = loadLDR(self.clouds[self.count])
            self.lidar_cloud = VtkPointCloud(lidar_data[:, :3], lidar_data[:,3])

            fren.RemoveActor(self.lidar_actor)
            self.lidar_actor = self.lidar_cloud.get_vtk_cloud(zMin=0, zMax=255)
            fren.AddActor(self.lidar_actor)

        if self.count == 0:
            fren.ResetCamera()
            fren.GetActiveCamera().Zoom(1.6)

        self.count += 1
        iren.GetRenderWindow().Render()

if __name__ == '__main__': 
    
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.SetSize(1280/2, 960/2)


    renderer = vtk.vtkRenderer()
    renderWindow.AddRenderer(renderer)


    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
    renderWindowInteractor.SetInteractorStyle(mouseInteractor)
    renderWindow.Render()
    cb = ImageGrabberCallback(sys.argv[1])
    
    renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
    timerId = renderWindowInteractor.CreateRepeatingTimer(1)
    renderWindowInteractor.Start()