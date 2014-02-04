#!/usr/bin/python
# -*- coding: utf-8 -*-

from VtkRenderer import *
from LidarTransforms import *
import numpy as np
import sys
import os
from VideoReader import VideoReader
import cv2
import cv

global count

class LDRGrabberCallback:

    def __init__(self):
        pass

    def execute(self, iren, event):
        global count
        t = time.time()

        (success, img) = reader.getNextFrame()
        if success == False:
            return

        (frame, ldr_file) = map_file.readline().rstrip().split(' ')
        if count % 3 == 0:
            count += 1
            return
        ldr_file = sys.argv[3] + '/' + ldr_file

        renderer = iren.GetRenderWindow().GetRenderers().GetFirstRenderer()

        pts = loadLDR(ldr_file)
        
        count += 1
        self.pointCloud = VtkPointCloud(pts[:, 0:3], pts[:, 3])

        if count > 2:
            renderer.RemoveActor(self.actor)
        

        self.actor = self.pointCloud.get_vtk_cloud(zMin=0, zMax=255)
        renderer.AddActor(self.actor)

        if count == 2:
            renderer.ResetCamera()

        iren.GetRenderWindow().Render()

        img = cv2.resize(img, (640, 480))
        cv2.imshow('display', img)
        cv2.waitKey(1)

        print time.time() - t


if __name__ == '__main__':
    reader = VideoReader(sys.argv[1])
    count = 1
    map_file = open(sys.argv[2], 'r')

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0., 0., 0.)
    renderer.ResetCamera()

    # Render Window
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(640, 480)

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




