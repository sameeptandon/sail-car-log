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

def load_ply(ply_file):
    """ Loads a ply file and returns an actor """
    reader = vtk.vtkPLYReader()
    reader.SetFileName(ply_file)
    reader.Update()

    ply_mapper = vtk.vtkPolyDataMapper()
    ply_mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(ply_mapper)
    return actor


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

def saveInterp(interp, num_lanes):
    out = {}
    out['num_lanes'] = np.array(num_lanes)
    for i in xrange(num_lanes):
        out['lane' + str(i)] = interp[:,:,i]

    print 'Saved multilane shifted points'
    np.savez('multilane_points', **out)

class Blockworld:

    def __init__(self):
        self.start = 0
        self.step = 5
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

        ml = MultiLane(sys.argv[3], sys.argv[4], 2, 2)

        ml.extendLanes()
        saveInterp(ml.interp, ml.rightLanes + ml.leftLanes)
        ml.filterLaneMarkings()

        print 'Adding filtered points'
        pts = ml.lanes.copy()
        raw_cloud = VtkPointCloud(pts[:, :3], pts[:, 4])
        raw_actor = raw_cloud.get_vtk_cloud(zMin=0, zMax=100)
        self.ren.AddActor(raw_actor)

        try:
            npz = np.load('cluster.npz')
            print 'Loading clusters from file'
            ml.lanes = npz['data']
            ml.times = npz['t']
        except IOError:
            print 'Clustering points'
            ml.clusterLanes()
            ml.saveLanes('cluster.npz')

        ml.sampleLanes()

        print 'Adding clustered points'
        clusters = ml.lanes.copy()
        cluster_cloud = VtkPointCloud(clusters[:, :3], clusters[:, -2])
        cluster_actor = cluster_cloud.get_vtk_cloud(zMin=0, zMax=4)
        cluster_actor.GetProperty().SetPointSize(10)
        self.ren.AddActor(cluster_actor)

        print 'Interpolating lanes'
        ml.interpolateLanes()
        interp_lanes = ml.interp_lanes.copy()
        interp_lanes_cloud = VtkPointCloud(interp_lanes[:, :3], interp_lanes[:, 3])
        interp_lanes_actor = interp_lanes_cloud.get_vtk_cloud(zMin=0, zMax=4)
        self.ren.AddActor(interp_lanes_actor)

        # ml.fixMissingPoints()
        # saveClusters(ml.lanes, ml.times, -1, 5)

        print 'Adding car'
        self.car = load_ply('../mapping/viz/gtr.ply')
        self.ren.AddActor(self.car)
        self.car.GetProperty().LightingOff()

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
        self.iren.Initialize()

        # Whether to write video
        self.record = False

        # Set up time
        self.iren.AddObserver('TimerEvent', self.update)
        self.timer = self.iren.CreateRepeatingTimer(100)

        # Add keypress event
        self.iren.AddObserver('KeyPressEvent', self.keyhandler)
        self.mode = 'ahead'

        self.iren.Start()

    def getCameraPosition(self):
        t = self.start + self.step * self.count
        if self.mode == 'ahead':
            position = self.imu_transforms[t, 0:3, 3]
            focal_point = self.imu_transforms[t + self.step, 0:3, 3]
        elif self.mode == 'behind':
            # FIXME Tune this
            position = self.imu_transforms[t - 0.3 * self.step, 0:3, 3]
            position[2] = position[2] + 0.15
            focal_point = self.imu_transforms[t + 0.2 * self.step, 0:3, 3]
            focal_point[2] = focal_point[2] - 0.2
        elif self.mode == 'above':
            position = self.imu_transforms[
                t - self.step, 0:3, 3] + np.array([0, 0, 75.0])
            focal_point = self.imu_transforms[t, 0:3, 3]
        elif self.mode == 'passenger':
            # TODO Not sure being inside mesh works...
            pass
        return position, focal_point

    def keyhandler(self, obj, event):
        key = obj.GetKeySym()
        if key == 'a':
            self.mode = 'above'
        elif key == 'b':
            self.mode = 'behind'
        elif key == 'd':
            self.mode = 'ahead'
        elif key == '0':
            self.count = 0
        else:
            pass

    def update(self, iren, event):
        # Transform the car
        t = self.start + self.step * self.count
        imu_transform = self.imu_transforms[t, :,:]
        transform = vtk_transform_from_np(imu_transform)
        transform.RotateZ(90)
        transform.Translate(-2, -3, -2)
        self.car.SetUserTransform(transform)

        # Set camera position
        fren = iren.GetRenderWindow().GetRenderers().GetFirstRenderer()
        cam = fren.GetActiveCamera()
        position, focal_point = self.getCameraPosition()
        cam.SetPosition(position)
        cam.SetFocalPoint(focal_point)
        cam.SetViewUp(0, 0, 1)
        fren.ResetCameraClippingRange()
        cam.SetClippingRange(0.1, 1600)
        iren.GetRenderWindow().Render()

        self.count += 1

if __name__ == '__main__':
    blockworld = Blockworld()
