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
from GPSReader import * 
from GPSTransforms import * 

global count
global lastTime


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
            lidar_pts = frame_cloud['lidar_pts']
            gps_pts = frame_cloud['gps_pts']
            img = frame_cloud['img']

            global count
            count += 1

            self.lidarPointCloud = VtkPointCloud(lidar_pts[:, 0:3], lidar_pts[:, 3])
            self.gpsPointCloud = VtkPointCloud(gps_pts[:,0:3],
                    gps_pts[:,0]-gps_pts[:,0] + 10)
            self.vtkImage = VtkImage(img)

            if count > 2:
                cloud_r.RemoveActor(self.lidar_actor)
                cloud_r.RemoveActor(self.gps_actor)
                image_r.RemoveActor(self.image_actor)
            self.lidar_actor = self.lidarPointCloud.get_vtk_cloud(zMin=0, zMax=255)
            self.gps_actor = self.gpsPointCloud.get_vtk_cloud()
            self.image_actor = self.vtkImage.get_vtk_image()

            cloud_r.AddActor(self.lidar_actor)
            cloud_r.AddActor(self.gps_actor)
            image_r.AddActor(self.image_actor)

            # Initially set the camera frame
            if count == 2:
                cloud_r.ResetCamera()
                image_r.ResetCamera()
                image_r.GetActiveCamera().Zoom(1.25)

            iren.GetRenderWindow().Render()

            #cv2.imshow('display', img)
            #cv2.waitKey(1)
            global lastTime
            #print time.time() - lastTime
            lastTime = time.time()

        except Queue.Empty:
            if self.manager.finished == True:
                return


class FrameCloudManager:

    def __init__(self, video_file, map_file):
        self.reader = VideoReader(video_file)
        self.ldr_map = loadLDRCamMap(map_file)
        self.queue = Queue.Queue()
        self.finished = False
        
        path, vfname = os.path.split(video_file)
        vidname = vfname.split('.')[0]
        gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
        gps_reader = GPSReader(gps_filename)
        self.GPSData = gps_reader.getNumericData()

    def loadNext(self):
        while self.finished == False:
            for t in range(5):
                (success, img) = self.reader.getNextFrame()
            if success == False:
                self.finished = True
                return

            #img = cv2.resize(img, (640, 480))
            img = cv2.pyrDown(img)
            
            framenum = self.reader.framenum
            if framenum >= len(self.ldr_map) or framenum >= self.GPSData.shape[0]:
                self.finished = True
                return
            roll_start = -deg2rad(self.GPSData[framenum,8])
            pitch_start = deg2rad(self.GPSData[framenum,7])
            yaw_start = -deg2rad(self.GPSData[framenum,9])

            base_R_to_i_from_w = R_to_i_from_w(roll_start, pitch_start, yaw_start)
            gps_pts = WGS84toENU(self.GPSData[framenum,1:4], self.GPSData[framenum-3*50:framenum+8*50,1:4])
            gps_pts = np.dot(base_R_to_i_from_w, gps_pts)
            gps_pts = np.dot(euler_matrix(0,-0.01,-0.04)[0:3,0:3], gps_pts)
            gps_pts = gps_pts.transpose()
            gps_pts[:,2] -= 1.5

                
            ldr_file = self.ldr_map[framenum]
            lidar_pts = loadLDR(ldr_file)

            self.queue.put({'img': img, 'lidar_pts': lidar_pts,
                'gps_pts': gps_pts})

            while self.queue.qsize() > 5:
                time.sleep(0.1)

def Keypress(obj, event):
    key = obj.GetKeySym()
    print key


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print """
        Usage:
            testPointCloudFrameCompare.py <video file>.avi <map_file>.out
        """
    else: 

        frame_cloud_manager = FrameCloudManager(sys.argv[1], sys.argv[2])


        global count
        count = 1
        global lastTime
        lastTime = 0

        cloud_r = vtk.vtkRenderer()
        cloud_r.SetBackground(0., 0., 0.)
        cloud_r.SetViewport(0,0,0.5,1.0)
        image_r = vtk.vtkRenderer()
        image_r.SetBackground(0., 0., 0.)
        image_r.SetViewport(0.5,0,1.0,1.0)
        image_r.SetInteractive(False)
        
        axes = vtk.vtkAxesActor()
        axes.AxisLabelsOff()
        cloud_r.AddActor(axes)

        # Render Window
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(cloud_r)
        renderWindow.AddRenderer(image_r)
        renderWindow.SetSize(1200, 600)

        # Interactor
        renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(renderWindow)
        mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
        renderWindowInteractor.SetInteractorStyle(mouseInteractor)
        renderWindow.Render()

        cb = LDRGrabberCallback(frame_cloud_manager)
        renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
        renderWindowInteractor.AddObserver('KeyPressEvent', Keypress)
        timerId = renderWindowInteractor.CreateRepeatingTimer(1)
        renderWindowInteractor.Start()
