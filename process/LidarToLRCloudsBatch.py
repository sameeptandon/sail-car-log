# usage: 
# python LidarToLRLanesBatch.py <rootdir> <output pickle name>



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
FRAME_WIN = 50*4
METER_WIN = 120

global left_data
global right_data


def integrateClouds(ldr_map, IMUTransforms, offset, num_steps, step):
    start = offset
    end = offset + num_steps*step


    trans_wrt_imu = IMUTransforms[start:end,0:3,3]
    for t in range(num_steps):
        fnum = offset+t*step
        if fnum>=len(ldr_map):
          break
        if fnum%500==0:
          print 'integrating pt clouds: '+str(fnum)+'/'+str(step*num_steps)
        
        data = loadLDR(ldr_map[fnum])
        # filter out the roof rack
        dist = np.sqrt(np.sum( data[:, 0:3] ** 2, axis = 1))

        # check out the commented out section below to figure out how this is filtering.
        data_filter_mask = (dist > 3)                  & \
                           (dist < 10)                  & \
                           (data[:,3] > 30)            & \
                           (data[:,0] > 0)             & \
                           (data[:,2] < -1.8)          & \
                           (data[:,2] > -1.9)          
        left_mask = data_filter_mask & (data[:,1] < 2.2) & (data[:,1] > 1.2)
        right_mask = data_filter_mask & (data[:,1] > -2.6) & (data[:,1] < -1.6)
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
    #left_data = left_data[left_data[:,3] > 60, :] # intensity filter
    #right_data = right_data[right_data[:,3] > 60, :] # intensity filter
    # map points are defined w.r.t the IMU position at time 0
    # each entry in map_data is (x,y,z,intensity,framenum). 
    total_num_frames = GPSData.shape[0]
    imu_transforms = IMUTransforms(GPSData)
    leftLaneData = np.zeros([total_num_frames, 3])
    rightLaneData = np.zeros([total_num_frames, 3])
    for frame in xrange(total_num_frames):
        if frame%500==0:
          print 'interpolating: '+str(frame)+'/'+str(total_num_frames)
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
        if ('kly' not in locals()) and len(X) <= 40:
          continue
  
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
        if ('kry' not in locals()) and len(X) <= 40:
          continue  
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
    step = 2
    rootdir = sys.argv[1]
    if rootdir[-1]=='/':
      rootdir = rootdir[0:-1] # remove trailing '/'
    path, directory = os.path.split(rootdir)
    targetfolder = '/scail/group/deeplearning/driving_data/640x480_Q50/' + directory + '/'

    for root, subfolders, files in os.walk(rootdir):
      files1 = filter(lambda z: '_gps.out' in z, files)
      if len(sys.argv)>2:
        files = filter(lambda z: sys.argv[2] in z, files1)
        if len(files1)==len(files):
          print 'warning: filter '+sys.argv[2]+' not found in files, including all files.'
      else:
        files = files1
      for f in files:
        gps_name = os.path.join(root,f)
        print 'gps: '+gps_name
        gps_reader = GPSReader(gps_name)
        GPSData = gps_reader.getNumericData()
        imu_transforms = IMUTransforms(GPSData)
        map_name = gps_name[0:-8]+'.map'
        print 'map: '+ map_name
        ldr_map = loadLDRCamMap(map_name)
        savename1 = os.path.join(targetfolder, (f[0:-8]+'_lidarmap.pickle')) 
        savename2 = os.path.join(targetfolder, (f[0:-8]+'_interp_lanes.pickle')) 
        print 'out: '+ savename1
        total_num_frames = GPSData.shape[0]
        num_fn = int(total_num_frames / step)
    
        left_data = [ ] 
        right_data = [ ] 
        start_fn = 0 # offset in frame numbers to start exporting data
        integrateClouds(ldr_map, imu_transforms, start_fn, num_fn, step)
        all_data = dict()
        all_data['left'] = np.row_stack(left_data)
        all_data['right'] = np.row_stack(right_data)
        #laneData = interpolatePoints(all_data['left'], all_data['right'], GPSData) 
        savefid = open(savename1,'w')
        pickle.dump(all_data, savefid)
        savefid.close()
