from Q50_config import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
from LaneMarkingHelper import mk2_to_mk1,mk1_to_mk2,get_transforms
import numpy as np
import cv2
from ArgParser import *
#from matplotlib import pyplot as plt
#import pylab as pl
import pickle
import string
import subprocess
WINDOW = 50*5
DIST_WINDOW = 80

def cloudToPixels(cam, pts_wrt_cam): 
    width = 4
    #(pix, J) = cv2.projectPoints(pts_wrt_cam.transpose(), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), cam['KK'], cam['distort'])

    #pix = pix.transpose()
    #pix = np.around(pix[:, 0, :])
    #pix = pix.astype(np.int32)
    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < cam['width']-1 - width/2)
    mask = np.logical_and(mask, pix[1,:] < cam['height']-1 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)
def localMapToPixels(map_data, imu_transforms_t, T_from_i_to_l, cam):
    # load nearby map frames
    pts_wrt_imu_0 = array(map_data[:,0:3]).transpose()
    pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0, np.ones((1,pts_wrt_imu_0.shape[1]))))
    # transform points from imu_0 to imu_t
    pts_wrt_imu_t = np.dot( np.linalg.inv(imu_transforms_t), pts_wrt_imu_0)
    # transform points from imu_t to lidar_t
    pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t);
    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = dot(R_to_c_from_l(cam), pts_wrt_camera_t.transpose())
    pts_wrt_camera_t = np.vstack((pts_wrt_camera_t,np.ones((1,pts_wrt_camera_t.shape[1]))))
    pts_wrt_camera_t = dot(cam['E'], pts_wrt_camera_t)
    pts_wrt_camera_t = pts_wrt_camera_t[0:3,:]
    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    return (pix, mask)

def localMapToPixelsTrajectory(imu_data, imu_transforms_t, T_from_i_to_l, cam, height=0):
    # load nearby map frames
    height_array = np.zeros([3,3])
    height_array[2,0]=-height
    aa = np.dot(imu_data[:,0:3,0:3], height_array)[:,:,0] # shift down in the self frame
    pts_wrt_imu_0 = (array(imu_data[:,0:3,3])+aa).transpose()
    pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0, np.ones((1,pts_wrt_imu_0.shape[1]))))
    # transform points from imu_0 to imu_t
    pts_wrt_imu_t = np.dot( np.linalg.inv(imu_transforms_t), pts_wrt_imu_0)
    # transform points from imu_t to lidar_t
    pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t);
    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = dot(R_to_c_from_l(cam), pts_wrt_camera_t.transpose())
    pts_wrt_camera_t = np.vstack((pts_wrt_camera_t,
        np.ones((1,pts_wrt_camera_t.shape[1]))))
    pts_wrt_camera_t = dot(cam['E'], pts_wrt_camera_t)
    pts_wrt_camera_t = pts_wrt_camera_t[0:3,:]
    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    return (pix, mask)

if __name__ == '__main__':

    rootdir = sys.argv[1]
    if rootdir[-1]=='/':
      rootdir = rootdir[0:-1] # remove trailing '/'
    
    cam_num = 601
    for root, subfolders, files in os.walk(rootdir):
      files1 = filter(lambda z: 'vail' not in z, files)
      files1 = filter(lambda z: '_gpsmark2.out' in z, files1)
      name_offset = len('_gpsmark2.out')
      if '4-2-14-monterey' in root:
        files1 = filter(lambda z: '1S_g' not in z, files1)
        files1 = filter(lambda z: '17N_c' not in z, files1)
      if '4-10-14' in root:
        files1 = filter(lambda z: '680s_a' not in z, files1)
        files1 = filter(lambda z: '237_a' not in z, files1)
      files1 = filter(lambda z: 'sandhill' not in z, files1)
      if len(sys.argv)>2:
        files = filter(lambda z: sys.argv[2] in z, files1)
        if len(files1)==len(files):
          print 'warning: filter '+sys.argv[2]+' not found in files, including all files.'
      else:
        files = files1
      print files
      for f in files:
        args = parse_args(root, f[0:-name_offset]+str(cam_num)+'.zip')
        if '.zip' in args['video']:
            cam_num = args['cam_num']
        else:
            cam_num = args['cam_num'] - 1

        print cam_num
        video_file = args['video']
        params = args['params']
        cam = params['cam'][cam_num]
        video_reader = VideoReader(video_file)
        absolute=False


        (imu_transforms_mk1,gps_data_mk1, gps_times_mk1) = get_transforms(args, 'mark1', absolute)
        (imu_transforms_mk2,gps_data_mk2, gps_times_mk2) = get_transforms(args, 'mark2', absolute)
        gpsname = args['gps_mark1']
        lidar_height = params['lidar']['height'] 
        T_from_i_to_l = np.linalg.inv(params['lidar']['T_from_l_to_i'])
   
        labelname = gpsname[0:-name_offset]+'_interp_lanes.pickle'
        #labelname = gpsname[0:-name_offset]+'_lidarmap.pickle'
        #labelname = string.replace(labelname, 'q50_data', '640x480_Q50') 
        #labelname = string.replace(labelname, 'sameep', '640x480_Q50') 
        labelfid = open(labelname,'r') 

        new_vid_name = string.replace(video_file, 'q50_data', '640x480_label_videos_nodistort')
        #new_vid_name = string.replace(video_file, 'sameep', '640x480_label_videos_nodistort')
        new_vid_name = string.replace(new_vid_name, '.avi', '')
        print 'writing to '+ new_vid_name
        #writer = cv2.VideoWriter(new_vid_name,cv.CV_FOURCC('M','J','P','G'),50,(640,320))
        
        all_data = pickle.load(labelfid)
        labelfid.close()
        left_data = all_data['left']
        right_data = all_data['right']

        # map points are defined w.r.t the IMU position at time 0
        # each entry in map_data is (x,y,z,intensity,framenum). 

        blue = [255,0,0] 
        green = [0,255,0] 
        red = [0,0,255]
        cnt=1
        mk2_t = 0
        t = mk2_to_mk1(mk2_t, gps_times_mk1, gps_times_mk2)
        while t<imu_transforms_mk1.shape[0] and mk2_t<imu_transforms_mk2.shape[0]:
          # If we are using the zip reader, getFrame is super fast
          if '.zip' in args['video']:
            (success, I) = video_reader.getFrame(mk2_t)
          else:
            while video_reader.framenum <= mk2_t:
                (success, I) = video_reader.getNextFrame()
          gps_idx = np.argmin(np.abs(gps_times_mk1 - gps_times_mk2[mk2_t]))
          forward_near_idx = np.where(np.abs(imu_transforms_mk1[gps_idx:-1,:3,3] - imu_transforms_mk1[gps_idx,:3,3])>=DIST_WINDOW)
          if forward_near_idx is not None and len(forward_near_idx[0])>0:
            WINDOW = forward_near_idx[0][0] # first future timestamp that is out of dist window range
          else:
            WINDOW = 1

          if t>=imu_transforms_mk1.shape[0]-WINDOW-1 or mk2_t>imu_transforms_mk2.shape[0]-WINDOW/4-1:
            mk2_t = 0
            t = mk2_to_mk1(mk2_t, gps_times_mk1, gps_times_mk2)
            break
          if (t-9)%100==0:
            print str(t)+'/'+str(left_data.shape)
          #left_data_copy = array(left_data[t+1:t+WINDOW+1, :]);
          #right_data_copy = array(right_data[t+1:t+WINDOW+1, :]);
          #imu_data_copy = array(imu_transforms_mk1[t+1:t+WINDOW+1, 0:3, 3])
          # reproject


          (pix, mask) = localMapToPixelsTrajectory(imu_transforms_mk1[t+1:t+WINDOW+1, :, :], imu_transforms_mk1[t,:,:], T_from_i_to_l, cam, height=lidar_height); 
          # find horizon
          #horizon = np.min(pix[1,mask])
          
          # draw
          for p in range(4):
            I[pix[1,mask]+p, pix[0,mask], :] = green
            I[pix[1,mask], pix[0,mask]+p, :] = green
            I[pix[1,mask]+p, pix[0,mask], :] = green
            I[pix[1,mask], pix[0,mask]+p, :] = green



          # reproject
          #(pix, mask) = localMapToPixels(left_data[t+1:t+WINDOW+1,:], imu_transforms_mk1[t,:,:], T_from_i_to_l, cam);
          (pix, mask) = localMapToPixels(left_data, imu_transforms_mk1[t,:,:], T_from_i_to_l, cam);
          #horizon = min(horizon, np.min(pix[1,mask]))
          # draw
          for p in range(4):
            I[pix[1,mask]+p, pix[0,mask], :] = green
            I[pix[1,mask], pix[0,mask]+p, :] = green
            I[pix[1,mask]+p, pix[0,mask], :] = green
            I[pix[1,mask], pix[0,mask]+p, :] = green
          # reproject
          #(pix, mask) = localMapToPixels(right_data[t+1:t+WINDOW+1,:], imu_transforms_mk1[t,:,:], T_from_i_to_l, cam); 
          (pix, mask) = localMapToPixels(right_data, imu_transforms_mk1[t,:,:], T_from_i_to_l, cam); 
          #horizon = min(horizon, np.min(pix[1,mask]))
          # draw
          for p in range(4):
            I[pix[1,mask]+p, pix[0,mask], :] = green
            I[pix[1,mask], pix[0,mask]+p, :] = green
            I[pix[1,mask]+p, pix[0,mask], :] = green
            I[pix[1,mask], pix[0,mask]+p, :] = green
          # draw horizon
          #I[horizon-1:horizon+2, :, :]=red
          
          I = cv2.resize(I, (640, 400))
          #writer.write(I)
          #cv2.imshow('vid', cv2.pyrDown(I))
          #cv2.waitKey(1)
          cv2.imwrite(new_vid_name+str(cnt)+'.png',I)
          cnt +=1
          mk2_t+=5
          t = mk2_to_mk1(mk2_t, gps_times_mk1, gps_times_mk2)
          #I = I[:,:,[2,1,0]]
          #pl.ion()
          #pl.imshow(I)
          ##pl.pause(.1)
          #pl.draw()
          #pl.clf()
          #pl.ioff()
        subprocess.call('ffmpeg -r 30 -i '+ new_vid_name+'%d.png -r 30 -s 640x400 -vb 20M '+ new_vid_name+'.avi', shell=True)
        subprocess.call('rm '+ new_vid_name+'*.png', shell=True)
