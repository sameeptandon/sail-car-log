from Q50_config import *
import sys, os
from GPSReader import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
import numpy as np
import cv2
from ArgParser import *
import bisect
from LidarTransforms import interp_transforms, transform_points_by_times

WINDOW = 2.0 * 1e6

def cloudToPixels(cam, pts_wrt_cam): 

    width = 4
    (pix, J)  = cv2.projectPoints(pts_wrt_cam.transpose(), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), cam['KK'], cam['distort'])

    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] < cam['height'] - 1 - width/2)
    mask = np.logical_and(mask, pix[0,:] < cam['width'] - 1 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

def lidarPtsToPixels(map_data, imu_transforms_t, T_from_i_to_l, cam):
    # Transform points back to imu_t
    pts_wrt_imu_0 = array(map_data[:,0:3]).transpose()
    pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0, 
        np.ones((1,pts_wrt_imu_0.shape[1]))))
    pts_wrt_imu_t = np.dot(np.linalg.inv(imu_transforms_t), pts_wrt_imu_0)
    #pts_wrt_imu_t = pts_wrt_imu_0

    # transform points from imu_t to lidar_t
    pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t)

    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = dot(R_to_c_from_l(cam),
            pts_wrt_camera_t.transpose())

    pts_wrt_camera_t = np.vstack((pts_wrt_camera_t,
        np.ones((1, pts_wrt_camera_t.shape[1]))))
    pts_wrt_camera_t = dot(cam['E'], pts_wrt_camera_t)
    pts_wrt_camera_t = pts_wrt_camera_t[0:3,:]

    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    return (pix, mask)



if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    video_file = args['video']
    video_reader = VideoReader(video_file)
    params = args['params']
    cam = params['cam'][cam_num-1]
    
    all_data = np.load(sys.argv[3])
    map_data = all_data['data']
    map_t = all_data['t']

    gps_reader_mark1 = GPSReader(args['gps_mark1'])
    gps_data_mark1 = gps_reader_mark1.getNumericData()
    gps_reader_mark2 = GPSReader(args['gps_mark2'])
    gps_data_mark2 = gps_reader_mark2.getNumericData()

    imu_transforms_mark1 = IMUTransforms(gps_data_mark1)
    imu_transforms_mark2 = IMUTransforms(gps_data_mark2)
    gps_times_mark1 = utc_from_gps_log_all(gps_data_mark1)
    gps_times_mark2 = utc_from_gps_log_all(gps_data_mark2)

    T_from_l_to_i = params['lidar']['T_from_l_to_i']
    T_from_i_to_l = np.linalg.inv(T_from_l_to_i)

    while True:
        (success, I) = video_reader.getNextFrame()
        fnum = video_reader.framenum
        if cam_num > 2:
            fnum *= 2 
        #t = utc_from_gps_log(gps_data_mark2[fnum, :])
        t = gps_times_mark2[fnum]
        fnum_mark1 = bisect.bisect(gps_times_mark1, t) - 1

        mask_window = (map_t < t + WINDOW) & (map_t > t - WINDOW);
        map_data_copy = array(map_data[mask_window, :]);
        (pix, mask) = lidarPtsToPixels(map_data_copy, imu_transforms_mark1[fnum_mark1, :, :], T_from_i_to_l, cam)

        intensity = map_data_copy[mask, 3]
        heat_colors = heatColorMapFast(intensity, 0, 100)
        for p in range(4):
            I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]

        cv2.imshow('vid', cv2.pyrDown(I))
        cv2.waitKey(50)
