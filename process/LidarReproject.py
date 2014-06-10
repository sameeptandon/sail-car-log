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
from ImageProcTools import processImage

WINDOW = 50*2

def cloudToPixels(cam, pts_wrt_cam): 

    width = 4
    (pix, J)  = cv2.projectPoints(pts_wrt_cam.transpose(), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), cam['KK'], cam['distort'])

    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

def lidarPtsToPixels(pts_wrt_lidar_t, imu_transforms_t, cam):
    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = dot(R_to_c_from_l(cam), 
            pts_wrt_camera_t.transpose())

    pts_wrt_camera_t = np.vstack((pts_wrt_camera_t,
        np.ones((1,pts_wrt_camera_t.shape[1]))))
    pts_wrt_camera_t = dot(cam['E'], pts_wrt_camera_t)
    pts_wrt_camera_t = pts_wrt_camera_t[0:3,:]

    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    return (pix, mask)


def trackbarOnchange(t, prev_t):
    if abs(t - prev_t) > 1:
        video_reader.setFrame(t)


if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    video_file = args['video']
    video_reader = VideoReader(video_file)
    params = args['params'] 
    cam = params['cam'][cam_num-1]
    gps_reader = GPSReader(args['gps_mark2'])
    gps_data = gps_reader.getNumericData()
    lidar_loader = LDRLoader(args['frames'])
    imu_transforms = IMUTransforms(gps_data)

    while True:
        (success, I) = video_reader.getNextFrame()
        print gps_data[video_reader.framenum,:]
        fnum = video_reader.framenum
        t = utc_from_gps_log(gps_data[fnum,:])
        data = lidar_loader.loadLDRWindow(t, 0.05)
        print data.shape
        (pix, mask) = lidarPtsToPixels(data[:,0:3].transpose(), imu_transforms[fnum,:,:], cam); 
        intensity = data[mask, 3]
        heat_colors = heatColorMapFast(intensity, 0, 100)
        for p in range(4):
            I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]
            I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]

        cv2.imshow('vid', I)
        cv2.waitKey(5)
