# usage: 
# python LidarIntegrator.py <dir> <basename><camnum>.avi <export name>.npz <optional additional flags such as --export or --full>

# to change the type of data exported, see the function integrateClouds 


from Q50_config import *
from ArgParser import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from LidarTransforms import *
from transformations import euler_matrix
import numpy as np
from ColorMap import *
import copy
import cv2
import pickle
FRAME_WIN = 50*2
METER_WIN = 30

global left_data
global right_data
global start_fn
global num_fn
global color_mode

actors =  []
clouds = [ ]
left_data = [ ] 
right_data = [ ] 

start_fn = 0 # offset in frame numbers to start exporting data
num_fn = 25 # number of frames to export. this is changed if --full is enabled
step = 5 # step between frames

def exportData():
        print 'exporting data'
        export_data = np.row_stack(all_data)
        print export_data
        print export_data.shape
        np.savez(sys.argv[3], data=export_data)
        print 'export complete'

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



def integrateClouds(ldr_map, IMUTransforms, offset, num_steps, step):
    start = offset
    end = offset + num_steps*step


    trans_wrt_imu = IMUTransforms[start:end,0:3,3]
    for t in range(num_steps):
        fnum = offset+t*step
        print fnum

        
        data = loadLDR(ldr_map[fnum])
        # filter out the roof rack
        dist = np.sqrt(np.sum( data[:, 0:3] ** 2, axis = 1))

        # check out the commented out section below to figure out how this is filtering.
        data_filter_mask = (dist > 3)                  & \
                           (dist < 10)                  & \
                           (data[:,3] > 40)            & \
                           (data[:,0] > 0)             & \
                           (data[:,2] < -1.8)          & \
                           (data[:,2] > -1.9)          
        left_mask = data_filter_mask & (data[:,1] < 2.9) & (data[:,1] > 1.2)
        right_mask = data_filter_mask & (data[:,1] > -2.9) & (data[:,1] < -1.2)
        left = data[left_mask, :]
        right = data[right_mask, :]
        """
        data = data[ dist > 3, :]
        # filter out on intensity
        data = data[ data[:,3] > 60 , :]
        # only points ahead of the car
        data = data[ data[:,0] > 0, :]
        # within 2.2 meters laterally of the lidar
        data = data[ np.abs(data[:,1]) < 2.2]

        # farther than 1.2 meters laterally of the lidar
        data = data[ np.abs(data[:,1]) > 1.2]

        # between -2.5 and -1.5 meters below the lidar
        data = data[ data[:,2] < -1.5]
        data = data[ data[:,2] > -2.5]
        """

        # transform data into IMU frame at time t
        lpts = left[:,0:3].transpose()
        lpts = np.vstack((lpts,np.ones((1,lpts.shape[1]))))
        #R = euler_matrix(rx,ry,rz)[0:3,0:3].transpose()
        #T_from_l_to_i[0:3,0:3] = R
        lpts = np.dot(T_from_l_to_i, lpts)
        # transform data into imu_0 frame
        lpts = np.dot(IMUTransforms[fnum,:,:], lpts);
        lpts = lpts.transpose()

        # for exporting purposes
        lpts_copy = array(lpts[:,0:3])
        lpts_copy = np.column_stack((lpts_copy, array(left[:,3])))
        lpts_copy = np.column_stack((lpts_copy, fnum*np.ones((lpts.shape[0],1))))
        left_data.append(lpts_copy)
        


        # transform data into IMU frame at time t
        rpts = right[:,0:3].transpose()
        rpts = np.vstack((rpts,np.ones((1,rpts.shape[1]))))
        #R = euler_matrix(rx,ry,rz)[0:3,0:3].transpose()
        #T_from_l_to_i[0:3,0:3] = R
        rpts = np.dot(T_from_l_to_i, rpts)
        # transform data into imu_0 frame
        rpts = np.dot(IMUTransforms[fnum,:,:], rpts);
        rpts = rpts.transpose()

        # for exporting purposes
        rpts_copy = array(rpts[:,0:3])
        rpts_copy = np.column_stack((rpts_copy, array(right[:,3])))
        rpts_copy = np.column_stack((rpts_copy, fnum*np.ones((rpts.shape[0],1))))
        right_data.append(rpts_copy)

def interpolatePoints(left_data, right_data, GPSData):
    print left_data.shape
    print right_data.shape
    left_data = left_data[left_data[:,3] > 60, :] # intensity filter
    right_data = right_data[right_data[:,3] > 60, :] # intensity filter
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
    return laneData


if __name__ == '__main__': 
    vfname = sys.argv[2]
    vidname = vfname.split('.')[0]
    vidname2 = vidname[:-1] + '2'
    video_filename2 = sys.argv[1] + '/' + vidname2 + '.avi'
    
    args = parse_args(sys.argv[1], sys.argv[2])

    gps_reader = GPSReader(args['gps'])
    cam1 = GetQ50CameraParams()[0] 
    cam2 = GetQ50CameraParams()[1] 
    gps_reader = GPSReader(args['gps'])
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    ldr_map = loadLDRCamMap(args['map'])
    savename = sys.argv[3]

    
    total_num_frames = GPSData.shape[0]
    start_fn = 0
    step = 5
    num_fn = int(total_num_frames / step)
    
    integrateClouds(ldr_map, imu_transforms, start_fn, num_fn, step)
    laneData = interpolatePoints(np.row_stack(left_data), np.row_stack(right_data), GPSData) 
    pickle.dump(laneData, open(savename,'w'))
    sys.exit(0)
