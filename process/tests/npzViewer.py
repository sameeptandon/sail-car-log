#!/usr/bin/python
# -*- coding: utf-8 -*-

from VtkRenderer import *
from LidarTransforms import *
import numpy as np
import sys
import os
import time

global count
count = 0


class LDRGrabberCallback:

    def __init__(self):
        pass

    def execute(self, obj, event):
        global count
        t = time.time()
        count += 1
        self.pointCloud = VtkPointCloud(data[:, 0:3], data[:, 3])
        actor = self.pointCloud.get_vtk_cloud(zMin=0, zMax=255)
        iren = obj
        if count > 1:
            iren.GetRenderWindow().GetRenderers().GetFirstRenderer().RemoveActor(self.actor)
        self.actor = actor
        iren.GetRenderWindow().GetRenderers().GetFirstRenderer().AddActor(self.actor)
        if count == 1:
            iren.GetRenderWindow().GetRenderers().GetFirstRenderer().ResetCamera()

        iren.GetRenderWindow().Render()


if __name__ == '__main__':
    items = np.load(sys.argv[1])
    data = items['data']
    times = items['t']
    
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0., 0., 0.)
    renderer.ResetCamera()

    # Render Window

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
