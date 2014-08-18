# usage: 
# python LidarToLRLanesBatch.py <rootdir> <output pickle name>



from Q50_config import *
from ArgParserNew import *
import sys, os
from GPSReaderNew import *
from GPSReprojection import *
from GPSTransforms import *
from LidarTransforms import *
from transformations import euler_matrix
import numpy as np
from ColorMap import *
import copy
import cv2
import pickle
from sets import Set
#from fileLock import FileLock
FRAME_WIN = 50
METER_WIN = 30

def findOverpassFrames(lidar_loader, scan_window,  GPSTime2, lidar_height, step=20, frames_fwd=50):
    frames = Set()
    cnt=0
    print 'finding overpasses... ' 
    while cnt<GPSTime2.shape[0]-frames_fwd:
        if cnt%1000==0:
          print '    '+str(cnt)+'/'+str(GPSTime2.shape[0])
        current_time = GPSTime2[cnt]
        # load points w.r.t lidar at current time
        data, t_data = lidar_loader.loadLDRWindow(current_time,scan_window)
        #print np.min(t_data)
        #print np.min(GPSTime)
        if data is None or data.shape[0]==0:
            cnt+=step
            continue
        # find points  above us.
        data_filter_mask = (data[:,1] > -2)                  & \
                           (data[:,1] < 2)                  & \
                           (data[:,3] < 15)            & \
                           (data[:,3] > -1)            & \
                           (data[:,2] < (lidar_height+10))          & \
                           (data[:,2] > (lidar_height+2))         
        if np.sum(data_filter_mask)>10:
          frames = frames.union(Set(range(int(cnt/10), int((cnt+frames_fwd)/10)))) 
        cnt+=step
    print 'Found '+ str(len(frames))+' frames around overpasses'
    return frames

if __name__ == '__main__': 
    step = 1
    rootdir = sys.argv[1]
    if rootdir[-1]=='/':
      rootdir = rootdir[0:-1] # remove trailing '/'
    path, directory = os.path.split(rootdir)
    savename = '/scail/group/deeplearning/driving_data/twangcat/schedules/hard_frames.pickle'
    cam_num = 2
    #name_offset = len('_gpsmark1.out')
    name_offset = len('_gpsmark1.out')
    for root, subfolders, files in os.walk(rootdir):
      files1 = filter(lambda z: 'vail' not in z, files)
      if '4-2-14-monterey' in root:
        files1 = filter(lambda z: '1S_g' not in z, files1)
      if '4-10-14-pleasanton' in root:
        files1 = filter(lambda z: '680s_a' not in z, files1)
        files1 = filter(lambda z: '237_a' not in z, files1)
      files1 = filter(lambda z: '_gpsmark1.out' in z, files1)
      files1 = filter(lambda z: 'sandhill' not in z, files1)
      if len(sys.argv)>2:
        files = filter(lambda z: sys.argv[2] in z, files1)
        if len(files1)==len(files):
          print 'warning: filter '+sys.argv[2]+' not found in files, including all files.'
      else:
        files = files1
      for f in files:


        args = parse_args(root, f[0:-name_offset]+str(cam_num)+'.avi')      
        params = args['params']                                   
        
        # 50Hz gps data                     
        #gps_name = args['gps']                                     
        gps_name = args['gps_mark1']       
        print args['video'] 
        # 20 Hz gps data
        gps_name2 = args['gps_mark2']                                     
        gps_reader2 = GPSReader(gps_name2)                           
        GPSData2 = gps_reader2.getNumericData()                     
        GPSTime2 = utc_from_gps_log_all(GPSData2)
        scan_window = 0.1
        lidar_loader = LDRLoader(args['frames'])
        lidar_height = params['lidar']['height']
        frames = dict()
        frames[args['video']] = findOverpassFrames(lidar_loader, scan_window, GPSTime2, lidar_height)
        if os.path.isfile(savename):
          #lock = FileLock(savename)
          #lock.acquire()
          #print lock.path, 'is locked for reading...'
          fid = open(savename, 'r')
          old_frames = pickle.load(fid) # frames already sitting in file
          fid.close()
          #lock.release()
        else:
          old_frames = dict()
        if len(frames[args['video']])>0:
          frames = dict(old_frames.items() + frames.items()) 
          #lock = FileLock(savename)
          #lock.acquire()
          #print lock.path, 'is locked for writing...'
          fid = open(savename, 'wb')
          pickle.dump(frames,fid)
          fid.close()
          #lock.release()
