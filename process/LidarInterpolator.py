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
import pickle
FRAME_WIN = 50*2
METER_WIN = 30


if __name__ == '__main__':
    #left_data = pickle.load(open(sys.argv[1],'r'))
    #right_data = pickle.load(open(sys.argv[2],'r'))
    #left_data = all_data['left']
    #right_data = all_data['right']
    left_data = np.load(sys.argv[1])
    right_data = np.load(sys.argv[2])
    left_data = left_data['data']
    right_data = right_data['data']
    left_data = left_data[left_data[:,3] > 60, :] # intensity filter
    right_data = right_data[right_data[:,3] > 60, :] # intensity filter
    gps_reader = GPSReader(sys.argv[3])
    savename = sys.argv[4]
    GPSData = gps_reader.getNumericData()
    # map points are defined w.r.t the IMU position at time 0
    # each entry in map_data is (x,y,z,intensity,framenum). 
    total_num_frames = GPSData.shape[0]
    imu_transforms = IMUTransforms(GPSData)
    leftLaneData = np.zeros([total_num_frames, 3])
    rightLaneData = np.zeros([total_num_frames, 3])
    for frame in xrange(total_num_frames):
        if frame%100==0:
          print frame
        imu_transforms_t = imu_transforms[frame,:,:]
        imu_pos = imu_transforms_t[0,3] # current x position in the 0th imu frame

        # left lane markings
        # only care within ~ 2 seconds window, so that not likely to have a loop that mess up linear interp
        mask_window = (left_data[:,4] < frame + FRAME_WIN) & (left_data[:,4] > frame-FRAME_WIN);
        # also only care about points within ~16 meters from current position
        mask_window = mask_window & (left_data[:,0] < imu_pos + METER_WIN) & (left_data[:,0] > imu_pos-METER_WIN);
        left_data_copy = array(left_data[mask_window, :]);

        leftx = imu_pos
        X = left_data_copy[:,0]
        Y = left_data_copy[:,1]
        Z = left_data_copy[:,2]
        A = np.vstack([X**2, X, np.ones(len(X))]).T
        if frame==0 or len(X)>40:
          kly, mly, cly = np.linalg.lstsq(A, Y)[0]
          klz, mlz, clz = np.linalg.lstsq(A, Z)[0]
        lefty = kly*(leftx**2)+mly*leftx+cly
        leftz = klz*(leftx**2)+mlz*leftx+clz
        leftLaneData[frame,:] = array([leftx, lefty, leftz]) 

        # right lane markings
        mask_window = (right_data[:,4] < frame + FRAME_WIN) & (right_data[:,4] > frame-FRAME_WIN);
        mask_window = mask_window & (right_data[:,0] < imu_pos + METER_WIN) & (right_data[:,0] > imu_pos-METER_WIN);
        right_data_copy = array(right_data[mask_window, :]);


        rightx = imu_pos
        X = right_data_copy[:,0]
        Y = right_data_copy[:,1]
        Z = right_data_copy[:,2]
        A = np.vstack([X**2, X, np.ones(len(X))]).T 
        if frame==0 or len(X)>40:
          kry, mry, cry = np.linalg.lstsq(A, Y)[0]
          krz, mrz, crz = np.linalg.lstsq(A, Z)[0]
        righty = kry*(rightx**2)+mry*rightx+cry
        rightz = krz*(rightx**2)+mrz*rightx+crz
        rightLaneData[frame,:] = array([rightx, righty, rightz]) 
    laneData = dict()
    laneData['left']=leftLaneData
    laneData['right']=rightLaneData

    pickle.dump(laneData, open(savename,'w'))



