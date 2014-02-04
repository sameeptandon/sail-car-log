#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import os
import Queue
import sys
import threading
from LidarTransforms import *
from VideoReader import VideoReader
from VtkRenderer import *

global count


class LDRGrabberCallback:

    def __init__(self, queue_manager):
        self.queue = queue_manager.queue
        self.manager = queue_manager
        bg_thread = threading.Thread(target=queue_manager.loadNext)
        bg_thread.daemon = True
        bg_thread.start()

    def execute(self, iren, event):
        try:

            frame_cloud = self.queue.get()
            pts = frame_cloud['pts']
            img = frame_cloud['img']

            global count
            count += 1

            self.pointCloud = VtkPointCloud(pts[:, 0:3], pts[:, 3])

            t = time.time()
            if count > 2:
                renderer.RemoveActor(self.actor)
            self.actor = self.pointCloud.get_vtk_cloud(zMin=0, zMax=255)
            renderer.AddActor(self.actor)

            # Initially set the camera frame

            if count == 2:
                renderer.ResetCamera()
            iren.GetRenderWindow().Render()

            cv2.imshow('display', img)
            cv2.waitKey(1)

            print "Display:",  time.time() - t
        except Queue.Empty:
            if self.manager.finished == True:
                return


class FrameCloudManager:

    def __init__(self, video_file, frame_folder, map_file):
        self.reader = VideoReader(video_file)
        self.map_file = open(map_file, 'r')
        self.frame_folder = frame_folder
        self.queue = Queue.Queue()
        self.finished = False

    def loadNext(self):
        while self.finished == False:
            t = time.time()
            (success, img) = self.reader.getNextFrame()
            if success == False:
                self.finished = True
                return

            img = cv2.resize(img, (640, 480))

            (frame, ldr_file) = \
                self.map_file.readline().rstrip().split(' ')
            ldr_file = self.frame_folder + '/' + ldr_file
            pts = loadLDR(ldr_file)

            self.queue.put({'img': img, 'pts': pts})
            print "Load:", time.time() - t


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print """
        Usage:
            testPointCloudFrameCompare.py <video file>.avi <frame folder>/ <map_file>.out
        """
    else: 

        frame_cloud_manager = FrameCloudManager(sys.argv[1], sys.argv[2],
                sys.argv[3])
        global count
        count = 1

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

        cb = LDRGrabberCallback(frame_cloud_manager)
        renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
        timerId = renderWindowInteractor.CreateRepeatingTimer(10)
        renderWindowInteractor.Start()
