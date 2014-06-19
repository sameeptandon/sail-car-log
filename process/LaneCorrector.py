#!/usr/bin/python
# -*- coding: utf-8 -*-
#Usage: MultiLane.py folder/ video.avi data.npz interpolated_lanes.pickle

from ArgParser import parse_args
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from MultiLaneGenerator import MultiLane
from Q50_config import LoadParameters
from VtkRenderer import VtkPointCloud, VtkBoundingBox
import numpy as np
from scipy.interpolate import UnivariateSpline
from scipy.spatial import distance, KDTree
from sklearn import cluster
import sys
from transformations import euler_from_matrix
import vtk

def vtk_transform_from_np(np4x4):
    vtk_matrix = vtk.vtkMatrix4x4()
    for r in range(4):
        for c in range(4):
            vtk_matrix.SetElement(r, c, np4x4[r, c])
    transform = vtk.vtkTransform()
    transform.SetMatrix(vtk_matrix)
    return transform

def get_transforms(args):
    """ Gets the IMU transforms for a run """
    gps_reader = GPSReader(args['gps'])
    gps_data = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(gps_data)
    return imu_transforms

def saveClusters(lanes, times, lane_idx, num_lanes):
    out = {}
    out['num_lanes'] = np.array(num_lanes)
    for i in xrange(num_lanes):
        mask = lanes[:, lane_idx] == i
        lane = lanes[mask]
        time = times[mask]

        lane = lane[:, :3]

        shifted = np.vstack((lane[1:, :], np.zeros((1, 3))))
        lane = np.hstack((lane, shifted))
        out['lane' + str(i)] = lane
        out['time' + str(i)] = time

    np.savez('multilane_points', **out)

class Blockworld:

    def __init__(self):
        self.start = 0
        self.step = 10
        self.end = self.step * 500
        self.count = 0

        self.ren = vtk.vtkRenderer()

        args = parse_args(sys.argv[1], sys.argv[2])

        # Transforms
        self.imu_transforms = get_transforms(args)
        self.trans_wrt_imu = self.imu_transforms[
            self.start:self.end:self.step, 0:3, 3]
        self.params = args['params']
        self.lidar_params = self.params['lidar']

        # print 'Adding raw points'
        # raw_npz = np.load(sys.argv[3])
        # pts = raw_npz['data']
        # raw_cloud = VtkPointCloud(pts[:, :3], pts[:, 3])
        # raw_actor = raw_cloud.get_vtk_cloud(zMin=0, zMax=100)
        # self.ren.AddActor(raw_actor)

        print 'Loading interpolated lanes'
        npz = np.load(sys.argv[4])
        num_lanes = int(npz['num_lanes'])

        interp_lanes = [None] * num_lanes
        interp_times = [None] * num_lanes
        interp_cloud = [None] * num_lanes
        interp_actor = [None] * num_lanes

        for i in xrange(num_lanes):
            interp_lanes[i] = npz['lane' + str(i)]
            interp_times[i] = npz['time' + str(i)]
            interp_cloud[i] = VtkPointCloud(interp_lanes[i][:, :3],
                                            np.ones((interp_lanes[i].shape[0]))
                                            * i)
            interp_actor[i] = interp_cloud[i].get_vtk_cloud(zMin=0, zMax=num_lanes+1)
            interp_actor[i].GetProperty().SetPointSize(7)
            self.ren.AddActor(interp_actor[i])
        
        print 'Rendering'
        self.ren.ResetCamera()

        self.win = vtk.vtkRenderWindow()
        self.ren.SetBackground(0, 0, 0)
        self.win.AddRenderer(self.ren)
        self.win.SetSize(800, 400)

        self.iren = vtk.vtkRenderWindowInteractor()
        self.iren .SetRenderWindow(self.win)
        mouseInteractor = vtk.vtkInteractorStyleTrackballCamera()
        self.iren.SetInteractorStyle(mouseInteractor)

        self.picker = vtk.vtkPointPicker()
        self.iren.SetPicker(self.picker)

        self.iren.Initialize()

        # Whether to write video
        self.record = False

        # Set up time
        self.iren.AddObserver('TimerEvent', self.update)
        self.timer = self.iren.CreateRepeatingTimer(100)

        # Add keypress event
        self.iren.AddObserver('KeyPressEvent', self.keyhandler)

        # Add mouse move even
        self.iren.AddObserver('LeftButtonPressEvent', self.mouseHandler)

        self.iren.Start()

    def getCameraPosition(self, t):
        position = self.imu_transforms[t - self.step, 0:3, 3] +\
                   np.array([-75.0, 0, 25.0])
        focal_point = self.imu_transforms[t, 0:3, 3]
        return position, focal_point

    def mouseHandler(self, iren, event=""):
        # renWin.HideCursor()
        x, y = iren.GetEventPosition()
        self.picker.Pick(x, y, 0, self.ren)
        p = self.picker.GetPickPosition()
        print p

    def keyhandler(self, obj, event):
        key = obj.GetKeySym()
        if key == 'a':
            self.mode = 'above'
        elif key == '0':
            self.count = 0
        elif key == 'r':
            self.record = ~self.record
            print self.record
            if self.record:
                self.startVideo()
            else:
                self.closeVideo()
        else:
            pass

    def update(self, iren, event):
        # Transform the car
        # t = self.start + self.step * self.count
        t = self.start + self.step

        imu_transform = self.imu_transforms[t, :,:]

        # Set camera position
        fren = iren.GetRenderWindow().GetRenderers().GetFirstRenderer()
        cam = fren.GetActiveCamera()
        position, focal_point = self.getCameraPosition(t)

        cam.SetPosition(position)
        cam.SetFocalPoint(focal_point)
        cam.SetViewUp(0, 0, 1)
        fren.ResetCameraClippingRange()
        cam.SetClippingRange(0.1, 1600)
        iren.GetRenderWindow().Render()

        if self.record:
            self.writeVideo()

        self.count += 1

    def startVideo(self):
        self.win2img = vtk.vtkWindowToImageFilter()
        self.win2img.SetInput(self.win)
        self.videoWriter = vtk.vtkFFMPEGWriter()
        self.videoWriter.SetFileName('multilane.avi')
        self.videoWriter.SetInputConnection(self.win2img.GetOutputPort())
        self.videoWriter.SetRate(10)  # 10 fps
        self.videoWriter.SetQuality(2)  # Highest
        self.videoWriter.SetBitRate(1000)  # kilobits/s
        self.videoWriter.SetBitRateTolerance(1000)
        self.videoWriter.Start()

    def writeVideo(self):
        self.win2img.Modified()
        self.videoWriter.Write()

    def closeVideo(self):
        self.videoWriter.End()
        self.videoWriter.Delete()
        self.win2img.Delete()

if __name__ == '__main__':
    blockworld = Blockworld()
