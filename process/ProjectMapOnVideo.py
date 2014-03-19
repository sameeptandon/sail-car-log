from Q50_config import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
import numpy as np
import cv2
from ArgParser import *

WINDOW = 50*5

def cloudToPixels(cam, pts_wrt_cam): 

    width = 4
    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)


if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])

    cam = GetQ50CameraParams()[cam_num - 1] 
    video_reader = VideoReader(args['video'])
    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    
    T_from_i_to_l = np.linalg.inv(T_from_l_to_i)
    #T_from_gps_to_l = [dot(T_from_i_to_l, np.linalg.inv(xform)) for xform in imu_transforms]
    #T_from_l_to_gps = [np.linalg.inv(xform) for xform in T_from_gps_to_l]

    all_data = np.load(sys.argv[3])
    map_data = all_data['data']
    map_data = map_data[map_data[:,3] > 60, :]
    # map points are defined w.r.t the IMU position at time 0
    # each entry in map_data is (x,y,z,intensity,framenum). 

    while True:
        for count in range(10):
            (success, I) = video_reader.getNextFrame()
        t = video_reader.framenum - 1
        mask_window = (map_data[:,4] < t + WINDOW) & (map_data[:,4] > t - WINDOW);
        map_data_copy = array(map_data[mask_window, :]);
        
        pts_wrt_imu_0 = array(map_data_copy[:,0:3]).transpose()
        pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0, 
            np.ones((1,pts_wrt_imu_0.shape[1]))))
        pts_wrt_imu_t = np.dot( np.linalg.inv(imu_transforms[t,:,:]), pts_wrt_imu_0)
        pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t);
        pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
        pts_wrt_camera_t = dot(R_to_c_from_l(cam), 
                pts_wrt_camera_t.transpose())
        (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

        intensity = map_data_copy[mask, 3]
        heat_colors = heatColorMapFast(intensity, 0, 100)
        I[pix[1,mask], pix[0,mask], :] = heat_colors[0,:,:]

        cv2.imshow('vid', I)
        cv2.waitKey(5)


