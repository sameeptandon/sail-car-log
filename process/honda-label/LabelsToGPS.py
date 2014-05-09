""" 
This script takes as input lane labels given by our detector and output GPS coordinates of the lane marking in WGS84 format.

python LabelsToGPS.py <video/gps identifier> <lane labels in px> """ 

import numpy as np
import pickle
import sys, time
from cv2 import imshow, waitKey
from scipy.io import loadmat, savemat
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from generate_lane_labels import *
from CameraReprojection import * 
from VideoReader import *
from CameraParams import *

def pixelToGPS(pixels, gps_dat, cam): 

    tr = GPSTransforms(gps_dat, cam)
    R_camera_pitch = euler_matrix(cam['rot_x'], cam['rot_y'], cam['rot_z'], 'sxyz')[0:3,0:3]
    roll_0 = -deg2rad(gps_dat[0,7]);
    pitch_0 = deg2rad(gps_dat[0,8]);
    yaw_0 = -deg2rad(gps_dat[0,9]+90);
    

    # convert pixel coordinates to pos w.r.t to camera using flat ground assumption
    pos_wrt_cam_t = pixelTo3d(pixels, cam)

    # from pos w.r.t cam at time t to pos w.r.t cam at time 0 
    pos_wrt_cam_0 = np.zeros((pixels.shape[0], 4))
    for t in range(min(pixels.shape[0], tr.shape[0])):
      pos_wrt_cam_0[t,:] = np.dot(tr[t, :, :], np.array([pos_wrt_cam_t[t,0], pos_wrt_cam_t[t,1], pos_wrt_cam_t[t,2], 1]))

    # pos w.r.t cam at time 0 -> pos w.r.t. imu at time 0
    T_to_i_from_c = np.eye(4);
    R_to_i_from_c = (np.dot(R_camera_pitch, cam['R_to_c_from_i'] )).transpose()
    T_to_i_from_c[0:3,0:3] = R_to_i_from_c
    pos_wrt_imu_0 = np.dot(T_to_i_from_c, pos_wrt_cam_0.transpose())

    # pos w.r.t imu at time 0 -> ENU w.r.t imu at time 0 
    T_to_w_from_i = np.eye(4);
    T_to_w_from_i[0:3,0:3] = R_to_i_from_w(roll_0, pitch_0, yaw_0).transpose();
    enu_wrt_imu_0 = np.dot(T_to_w_from_i, pos_wrt_imu_0).transpose();
  
    # ENU w.r.t imu at time 0 -> WGS84
    wgs84_pts = ENUtoWGS84(gps_dat[0,1:4], enu_wrt_imu_0[:,0:3]).transpose()
    return wgs84_pts

if __name__ == '__main__':
    video_filename = sys.argv[1]
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    video_reader = VideoReader(video_filename,num_splits=1)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()
    cam = getCameraParams()[cam_num - 1] 

    labels = loadmat(sys.argv[2])
    lp = labels['left']
    rp = labels['right']

    l_gps_coord = pixelToGPS(lp, gps_dat, cam);
    r_gps_coord = pixelToGPS(rp, gps_dat, cam);

    print "gps_dat: "
    print gps_dat[:, 1:4]
    print "l_gps_dat: "
    print l_gps_coord
    print "r_gps_dat: "
    print r_gps_coord

    savemat('gps_coord.mat', dict({'l': l_gps_coord, 
                                   'r': r_gps_coord,
                                   'm': gps_dat[:, 1:4]}))
  
