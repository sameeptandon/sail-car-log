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
import scipy.interpolate
FRAME_WIN = 50
METER_WIN = 30

global left_data
global right_data


def integrateClouds(ldr_map, imuTransforms, offset, num_steps, step, T_from_l_to_i, lidar_height):
    start = offset
    end = offset + num_steps*step


    trans_wrt_imu = imuTransforms[start:end,0:3,3]
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
                           (data[:,3] > 70)            & \
                           (data[:,0] > 0)             & \
                           (data[:,2] < -(lidar_height-0.05))          & \
                           (data[:,2] > -(lidar_height+0.05))          
        left_mask = data_filter_mask & (data[:,1] < 2.2) & (data[:,1] > 1.2)
        #left_mask = data_filter_mask & (data[:,1] < 2.6) & (data[:,1] > 1.2)
        right_mask = data_filter_mask & (data[:,1] > -2.6) & (data[:,1] < -1.6)
        #right_mask = data_filter_mask & (data[:,1] > -2.6) & (data[:,1] < -1.2)
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
        lpts = np.dot(imuTransforms[fnum,:,:], lpts);
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
        rpts = np.dot(imuTransforms[fnum,:,:], rpts);
        rpts = rpts.transpose()

        # for exporting purposes
        rpts_copy = array(rpts[:,0:3])
        rpts_copy = np.column_stack((rpts_copy, array(right[:,3])))
        rpts_copy = np.column_stack((rpts_copy, fnum*np.ones((rpts.shape[0],1))))
        right_data.append(rpts_copy)

def interpolatePoints(left_data, right_data, imu_transforms, sideways, lidar_height=1.85):
    print left_data.shape
    print right_data.shape

    height_array = np.zeros([3,3])
    height_array[2,0]=-lidar_height
    aa = np.dot(imu_transforms[:,0:3,0:3], height_array)[:,:,0] # shift down in the self frame
    imu_transforms[1:,0:3,3] += aa[1:,:]

    #left_data = left_data[left_data[:,3] > 60, :] # intensity filter
    #right_data = right_data[right_data[:,3] > 60, :] # intensity filter
    # map points are defined w.r.t the IMU position at time 0
    # each entry in map_data is (x,y,z,intensity,framenum). 
    total_num_frames = imu_transforms.shape[0]
    leftLaneData = np.ones([total_num_frames, 3])*1e6
    rightLaneData = np.ones([total_num_frames, 3])*1e6
    for frame in xrange(total_num_frames):
        if frame%500==0:
          print 'collecting L/R lane pts: '+str(frame)+'/'+str(total_num_frames)
        imu_transforms_t = imu_transforms[frame,:,:]
        imu_pos = imu_transforms_t[0,3] # current x position in the 0th imu frame
        # only care within ~ 2 seconds window, so that not likely to have a loop that mess up linear interp
        # also only care about points within ~16 meters from current position
        mask_window = (left_data[:,4] < frame + FRAME_WIN) & (left_data[:,4] > frame-FRAME_WIN);
        mask_window = mask_window & (left_data[:,0] < imu_pos + METER_WIN) & (left_data[:,0] > imu_pos-METER_WIN);
        left_data_copy = array(left_data[mask_window, 0:3]);
        if left_data_copy.shape[0]>0:
          # find the 'left lane' point that closest to the current self-position
          l_distances = np.cross(left_data_copy - imu_transforms_t[0:3,3].transpose(), sideways[frame,:], axisa=1)
          #l_distances = left_data_copy - imu_transforms_t[0:3,3].transpose()
          #l_angles = np.dot(l_distances, sideways[frame,:])
          l_distances = np.sqrt((l_distances ** 2).sum(-1))
          #l_angles = l_angles/l_distances
          #idx = np.argmin(l_distances)
          if np.min(l_distances)<=0.2: #and np.abs(l_angles[idx])<0.08: # do nothing if it's too far.
            # compute the relative position of the left lane marking exactly on my side
            leftLaneData[frame,:] = left_data_copy[np.argmin(l_distances),:] - imu_transforms_t[0:3,3]
        
          mask_window = (right_data[:,4] < frame + FRAME_WIN) & (right_data[:,4] > frame-FRAME_WIN);
          mask_window = mask_window & (right_data[:,0] < imu_pos + METER_WIN) & (right_data[:,0] > imu_pos-METER_WIN);
        right_data_copy = array(right_data[mask_window, 0:3]);
        if right_data_copy.shape[0]>0:
          # find the 'right lane' point that closest to the current self-position
          r_distances = np.cross(right_data_copy - imu_transforms_t[0:3,3].transpose(), sideways[frame,:], axisa=1)
          #r_distances = right_data_copy - imu_transforms_t[0:3,3].transpose()
          #r_angles = np.dot(r_distances, sideways[frame,:])
          r_distances = np.sqrt((r_distances ** 2).sum(-1))
          #r_angles = r_angles/r_distances
          idx = np.argmin(r_distances)
          if np.min(r_distances)<=0.2: #and np.abs(r_angles[idx])<0.08:
            # compute the relative position of the right lane marking exactly on my side
            rightLaneData[frame,:] = right_data_copy[np.argmin(r_distances),:] - imu_transforms_t[0:3,3]
   
    
    print 'interpolating...'
    # perform interpolation on the relative positions, then add back to the absolute imu positions
    all_time = np.arange(0, total_num_frames)
    left_time_array = np.where(leftLaneData[:, 0] < 5e5)[0]
    print left_time_array.shape
    right_time_array = np.where(rightLaneData[:, 0] < 5e5 )[0]
    print right_time_array.shape
    polynomial_fit=1
    smoothing = 15
    spline_left_x = scipy.interpolate.UnivariateSpline(left_time_array, leftLaneData[left_time_array, 0], k=polynomial_fit, s=smoothing)
    spline_left_y = scipy.interpolate.UnivariateSpline(left_time_array, leftLaneData[left_time_array, 1], k=polynomial_fit, s=smoothing)
    spline_left_z = scipy.interpolate.UnivariateSpline(left_time_array, leftLaneData[left_time_array, 2], k=polynomial_fit, s=smoothing)
    spline_right_x = scipy.interpolate.UnivariateSpline(right_time_array, rightLaneData[right_time_array, 0],k=polynomial_fit, s=smoothing)
    spline_right_y = scipy.interpolate.UnivariateSpline(right_time_array, rightLaneData[right_time_array, 1],k=polynomial_fit, s=smoothing)
    spline_right_z = scipy.interpolate.UnivariateSpline(right_time_array, rightLaneData[right_time_array, 2],k=polynomial_fit, s=smoothing)

    output_left = np.copy(imu_transforms[:,0:3,3]);
    output_left[:,0] += smoothData(spline_left_x(all_time));
    output_left[:,1] += smoothData(spline_left_y(all_time));
    output_left[:,2] += smoothData(spline_left_z(all_time));

    output_right = np.copy(imu_transforms[:,0:3,3]);
    output_right[:,0] += smoothData(spline_right_x(all_time));
    output_right[:,1] += smoothData(spline_right_y(all_time));
    output_right[:,2] += smoothData(spline_right_z(all_time));
    









 
    laneData = dict()
    laneData['left']=output_left
    laneData['right']=output_right
    return laneData


if __name__ == '__main__': 
    step = 2
    rootdir = sys.argv[1]
    if rootdir[-1]=='/':
      rootdir = rootdir[0:-1] # remove trailing '/'
    path, directory = os.path.split(rootdir)
    targetfolder = '/scail/group/deeplearning/driving_data/640x480_Q50/' + directory + '/'
    cam_num = 2
    for root, subfolders, files in os.walk(rootdir):
      files1 = filter(lambda z: 'vail' not in z, files)
      if '4-2-14-monterey' in root:
        files1 = filter(lambda z: '1S_g' not in z, files1)
      if '4-10-14-pleasanton' in root:
        files1 = filter(lambda z: '680s_a' not in z, files1)
        files1 = filter(lambda z: '237_a' not in z, files1)
      files1 = filter(lambda z: '_gps.out' in z, files1)
      if len(sys.argv)>2:
        files = filter(lambda z: sys.argv[2] in z, files1)
        if len(files1)==len(files):
          print 'warning: filter '+sys.argv[2]+' not found in files, including all files.'
      else:
        files = files1
      for f in files:


        args = parse_args(root, f[0:-8]+str(cam_num)+'.avi')      
        params = args['params']                                   
        cam = params['cam'][cam_num-1]                            
        gps_name = args['gps']                                     
        gps_reader = GPSReader(gps_name)                           
        GPSData = gps_reader.getNumericData()                     
        imu_transforms = IMUTransforms(GPSData)                   
                                                                  
        T_from_l_to_i = params['lidar']['T_from_l_to_i']
        T_from_i_to_l = np.linalg.inv(T_from_l_to_i)
        lidar_height = params['lidar']['height']



        print 'gps: '+gps_name
        map_name = gps_name[0:-8]+'.map'
        print 'map: '+ map_name
        ldr_map = loadLDRCamMap(map_name)
        savename1 = os.path.join(targetfolder, (f[0:-8]+'_lidarmap.pickle')) 
        savename2 = os.path.join(targetfolder, (f[0:-8]+'_interp_lanes.pickle')) 
        print 'out: '+ savename2
        total_num_frames = GPSData.shape[0]
        num_fn = int(total_num_frames / step)
        
        velocities = GPSData[:,4:7]
        velocities[:,[0, 1]] = velocities[:,[1, 0]]
        vel_start = ENU2IMUQ50(np.transpose(velocities), GPSData[0,:])
        sideways_start = np.cross(vel_start.transpose(), imu_transforms[:,0:3,2], axisa=1, axisb=1, axisc=1) # sideways vector wrt starting imu frame
        sideways_start /= np.sqrt((sideways_start ** 2).sum(1))[...,np.newaxis]



 
        left_data = [ ] 
        right_data = [ ] 
        start_fn = 0 # offset in frame numbers to start exporting data
        integrateClouds(ldr_map, imu_transforms, start_fn, num_fn, step, T_from_l_to_i, lidar_height)
        all_data = dict()
        all_data['left'] = np.row_stack(left_data)
        all_data['right'] = np.row_stack(right_data)
        savefid1 = open(savename1,'w')
        pickle.dump(all_data, savefid1)
        savefid1.close()
        infid= open(savename1,'r')
        all_data = pickle.load(infid)
        infid.close()
        laneData = interpolatePoints(all_data['left'], all_data['right'], imu_transforms, sideways_start, lidar_height = lidar_height) 
        savefid = open(savename2,'w')
        pickle.dump(laneData, savefid)
        savefid.close()
