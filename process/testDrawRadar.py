#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys, os
from VtkRenderer import *
import numpy as np
from LidarTransforms import *


class ImageGrabberCallback:

    def __init__(self, map_file):
        self.map_file = map_file

        self.lidar_actor = None
        self.radar_actor = None

        self.clouds = loadLDRCamMap(map_file)
        self.rdr_pts = loadRDRCamMap(map_file)
        self.count = 0
    def execute(self, iren, event):
        radar_data = loadRDR(self.rdr_pts[self.count])
        self.radar_cloud = VtkPointCloud(radar_data[:, :3], radar_data[:,5])
        
        lidar_data = loadLDR(self.clouds[self.count])
        self.lidar_cloud = VtkPointCloud(lidar_data[:, :3], lidar_data[:,3])

        fren = iren.GetRenderWindow().GetRenderers().GetFirstRenderer()

        fren.RemoveActor(self.radar_actor)
        self.radar_actor = self.radar_cloud.get_vtk_cloud()
        self.radar_actor.GetProperty().SetPointSize(5)
        fren.AddActor(self.radar_actor)

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
    timerId = renderWindowInteractor.CreateRepeatingTimer(10)
    renderWindowInteractor.Start()