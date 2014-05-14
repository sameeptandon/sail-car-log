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
from ArgParser import *
from StereoCompute import *  

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
            img = frame_cloud['img']
            stereo_pts = frame_cloud['stereo_pts']

            global count
            count += 1

            self.lidarPointCloud = VtkPointCloud(lidar_pts[:, 0:3], lidar_pts[:, 3])
            self.stereoPointCloud = VtkPointCloud(stereo_pts[:,0:3], 10+ 0*stereo_pts[:,0])
            self.vtkImage = VtkImage(img)

            if count > 2:
                cloud_r.RemoveActor(self.lidar_actor)
                cloud_r.RemoveActor(self.stereo_actor)
                image_r.RemoveActor(self.image_actor)
            self.lidar_actor = self.lidarPointCloud.get_vtk_cloud(zMin=0, zMax=255)
            self.stereo_actor = self.stereoPointCloud.get_vtk_cloud(zMin=-10, zMax=10)
            self.image_actor = self.vtkImage.get_vtk_image()

            cloud_r.AddActor(self.lidar_actor)
            cloud_r.AddActor(self.stereo_actor)
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

    def __init__(self, args):
        self.args = args
        self.params = args['params']
        self.reader_left = VideoReader(args['video'])
        self.reader_right = VideoReader(args['opposite_video'])
        self.ldr_map = loadLDRCamMap(args['map'])
        self.queue = Queue.Queue()
        self.finished = False
        
    def loadNext(self):
        while self.finished == False:
            for t in range(5):
                (success, imgL) = self.reader_left.getNextFrame()
                (success, imgR) = self.reader_right.getNextFrame()
            if success == False:
                self.finished = True
                return

            #(disp, Q, R1, R2) = doStereo(imgL, imgR, self.params)
            #cv2.imshow('disp', disp)
            #print Q
            #stereo_points = get3dPoints(disp,Q)
            #stereo_points = stereo_points[disp > 5, :]
            (R1, R2, P1, P2, Q, size1, size2, map1x, map1y, map2x, map2y) = computeStereoRectify(self.params)
            stereo_points = np.load(sys.argv[3] + '/3d_' + str(self.reader_left.framenum) + '.npz')['data']
            print stereo_points
            stereo_points = stereo_points.transpose()
            stereo_points = np.dot(R1.transpose(), stereo_points)
            print np.amax(stereo_points, axis=1)
            print np.amin(stereo_points, axis=1)
            stereo_points = np.vstack((stereo_points,
                np.ones((1,stereo_points.shape[1]))))
            print stereo_points.shape
            stereo_points = dot(np.linalg.inv(self.params['cam'][0]['E']), stereo_points)
            stereo_wrt_lidar = np.dot(R_to_c_from_l(self.params['cam'][0]).transpose(), stereo_points[0:3,:])
            stereo_wrt_lidar = stereo_wrt_lidar.transpose()
            stereo_wrt_lidar = stereo_wrt_lidar[:,0:3] - self.params['cam'][0]['displacement_from_l_to_c_in_lidar_frame']


            #img = cv2.resize(img, (640, 480))
            imgL = cv2.pyrDown(imgL)
            #cv2.imshow('disparity', cv2.pyrDown(disp)/64.0)
            
            framenum = self.reader_left.framenum
            if framenum >= len(self.ldr_map):
                self.finished = True
                return
            
            ldr_file = self.ldr_map[framenum]
            lidar_pts = loadLDR(ldr_file)
            
            self.queue.put({'img': imgL, 'lidar_pts': lidar_pts,
                'stereo_pts': stereo_wrt_lidar})

            """
                while self.queue.qsize() > 5:
                    time.sleep(0.1)
            """

def Keypress(obj, event):
    key = obj.GetKeySym()
    print key


if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    params = args['params']
    assert(cam_num == 1)

    frame_cloud_manager = FrameCloudManager(args)

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
