import bisect
from collections import deque
from colorsys import hsv_to_rgb
import glob
import math
import multiprocessing
import os
import sys
import time
import urllib
import numpy as np
from scipy.spatial import cKDTree

from ArgParser import parse_args
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms, absoluteTransforms
from LidarTransforms import R_to_c_from_l, utc_from_gps_log_all
from MapReproject import lidarPtsToPixels

def get_transforms(args, mark='mark1', absolute=False):
    """ Gets the IMU transforms for a run """
    gps_reader = GPSReader(args['gps_' + mark])
    gps_data = gps_reader.getNumericData()
    gps_times = utc_from_gps_log_all(gps_data)
    if absolute:
        imu_transforms = absoluteTransforms(gps_data)
    else:
        imu_transforms = IMUTransforms(gps_data)

    return imu_transforms, gps_data, gps_times

def mk2_to_mk1(mk2_idx, gps_times_mk1, gps_times_mk2):
    t = gps_times_mk2[mk2_idx]
    mk1_idx = bisect.bisect(gps_times_mk1, t) - 1
    return mk1_idx

class BackProjector(object):
    def __init__(self, args):
        params = args['params']
        cam_num = args['cam_num'] - 1
        cam = params['cam'][cam_num]

        K = cam['KK']

        self.K_ = np.linalg.inv(K)
        self.R_ = np.linalg.inv(R_to_c_from_l(cam))
        self.t_ = -cam['displacement_from_l_to_c_in_lidar_frame']
        self.T_from_l_to_i = params['lidar']['T_from_l_to_i']

    def fromHomogenous(self, x):
        x /= x[-1]
        return x[:-1]

    def toHomogenous(self, x):
        return np.hstack((x, np.array(1)))

    def backProject(self, imu_xform, pt=None):
        if pt == None:
            pt = np.array([0, 0, 0])

        u = self.K_.dot(pt)
        u_lidar = self.toHomogenous(self.R_.dot(u) + self.t_)
        u_imu_0 = self.T_from_l_to_i.dot(u_lidar)
        u_imu_t = imu_xform.dot(u_imu_0)

        return self.fromHomogenous(u_imu_t)

    def getIntersection(self, l0, l, n, p0):
        alpha = np.inner((p0 - l0), n) / np.inner(l, n)
        return l0 + alpha * l

class DataTree(object):

    def __init__(self, pts):
        self.pts = pts
        self.tree = cKDTree(self.pts)

class VTKCloudTree(DataTree):

    def __init__(self, cloud, actor):
        super(VTKCloudTree, self).__init__(cloud.xyz)
        self.cloud = cloud
        self.actor = actor

class VTKPlaneTree(DataTree):

    def __init__(self, xyz, planes, actors):
        super(VTKPlaneTree, self).__init__(xyz)
        self.planes = planes
        self.actors = actors


def projectPointsOnImg(I, pts_tree, imu_transforms_mk1, t,
                       T_from_i_to_l, cam_params, color):
    car_pos = imu_transforms_mk1[t, 0:3, 3]

    # Project the points onto the image
    tree = pts_tree.tree
    (d, closest_idx) = tree.query(car_pos)

    # Find all the points nearby
    nearby_idx = np.array(tree.query_ball_point(car_pos, r=100.0))

    if nearby_idx.shape[0] > 0:
        lane = pts_tree.pts[nearby_idx, :3]
        # Reverse the color (RGB->BGR)
        if lane.shape[0] > 0:
            xform = imu_transforms_mk1[t, :,:]
            pix, mask = lidarPtsToPixels(lane, xform, T_from_i_to_l,
                                         cam_params)
            for p in range(4):
                I[pix[1, mask]+p, pix[0, mask], :] = color
                I[pix[1, mask], pix[0, mask]+p, :] = color
                I[pix[1, mask]-p, pix[0, mask], :] = color
                I[pix[1, mask], pix[0, mask]-p, :] = color

    return I
