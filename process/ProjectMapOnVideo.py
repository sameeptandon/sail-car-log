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

WINDOW = 50*2.5

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

def localMapToPixels(map_data, imu_transforms_t, T_from_i_to_l, cam):
    # load nearby map frames
    pts_wrt_imu_0 = array(map_data[:,0:3]).transpose()
    pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0, 
        np.ones((1,pts_wrt_imu_0.shape[1]))))
    # transform points from imu_0 to imu_t
    pts_wrt_imu_t = np.dot( np.linalg.inv(imu_transforms_t), pts_wrt_imu_0)
    # transform points from imu_t to lidar_t
    pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t);
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

    params = args['params'] 
    cam = params['cam'][cam_num-1]
    video_reader = VideoReader(video_file)
    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    
    T_from_i_to_l = np.linalg.inv(params['lidar']['T_from_l_to_i'])

    all_data = np.load(sys.argv[3])
    map_data = all_data['data']
    #map_data = map_data[map_data[:,3] > 60, :]
    # map points are defined w.r.t the IMU position at time 0
    # each entry in map_data is (x,y,z,intensity,framenum). 

    print "Hit 'q' to quit"
    trackbarInit = False
    while True:
        for count in range(20):
            (success, I) = video_reader.getNextFrame()

        if not success:
            break

        t = video_reader.framenum - 1
        print t
        mask_window = (map_data[:,4] < t + WINDOW) & (map_data[:,4] > t );
        map_data_copy = array(map_data[mask_window, :]);

        # reproject
        (pix, mask) = localMapToPixels(map_data_copy, imu_transforms[t,:,:], T_from_i_to_l, cam); 

        # draw 
        intensity = map_data_copy[mask, 3]
        heat_colors = heatColorMapFast(intensity, 0, 100)
        for p in range(4):
            I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]
            I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]

        cv2.imshow(video_file, cv2.pyrDown(I))
        if not trackbarInit:
            cv2.createTrackbar('trackbar', video_file, 0, int(video_reader.total_frame_count), lambda x: trackbarOnchange(x, t))
            trackbarInit = True
        else:
            cv2.setTrackbarPos('trackbar', video_file, t)

        keycode = cv2.waitKey(1)
        if keycode == 113:
            break

    print 'Played %d frames' % t
