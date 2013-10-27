import pickle
import sys, os
from GPSReader import *
from VideoReader import *
from WGS84toENU import *
from GPSReprojection import *
from transformations import euler_matrix
from numpy import array, dot, zeros, around, divide, nonzero, float32, maximum
import numpy as np
from cv2 import imshow, waitKey, resize, warpPerspective, getPerspectiveTransform, transpose, Canny, namedWindow
import cv
import cv2
import time
from scipy.io import savemat


left_frames = []
frame_data = []
frameWaitTime = 100

if __name__ == '__main__':
  video_filename = sys.argv[1]
  existing_lanes = []
  path, vfname = os.path.split(video_filename)
  vidname = vfname.split('.')[0]
  cam_num = int(vidname[-1])
  gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
  out_name = sys.argv[2]
  display = True
  if '--quiet' in sys.argv:
      display = False


  cv2.namedWindow('video')
  num_imgs_fwd = 200; 
  video_reader = VideoReader(video_filename, num_splits=1)
  gps_reader = GPSReader(gps_filename)
  gps_dat = gps_reader.getNumericData()

  cam = pickle.load(open('cam_params.pickle', 'rb'))[cam_num - 1]

  framenum = 0
  lastTime = time.time()
  video_reader.setFrame(framenum)

  skip_frame = 10
  default_offset = 60

  left_present = True
  right_present = True

  while True:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      print framenum, 'finished'
      break

    if framenum % skip_frame != 0:
      existing_lanes.append([left_present, right_present])
      continue

    if framenum % 150 == 0 and False:
        r = np.arange(9,len(frame_data)*skip_frame,skip_frame)
        export_data = -1*np.ones((len(frame_data)*skip_frame+1,2))
        export_data[r,:] = frame_data
        left_data = -1*np.ones((len(left_frames)*skip_frame+1,2))
        left_data[r, :] = left_frames
        savemat(out_name, dict(left=left_data,right=export_data))

    I = resize(I, (320, 240))
    key = ''
    if display:
        imshow('video',I)
        key = (waitKey(frameWaitTime) & 255)
    frameWaitTime = max(int(frameWaitTime / 1.5), 5)

    if key == ord('q'):
      break;

    if key == ord('a'):
        left_present = True
        right_present = False
    if key == ord('s'):
        left_present = True
        right_present = True
    if key == ord('d'):
        left_present = False
        right_present = True
    if key == ord('w'):
        left_present = False
        right_present = False
    
    existing_lanes.append([left_present, right_present])

    currentTime = time.time();
    if (currentTime - lastTime > 10):
        lastTime = currentTime
        print framenum

  existing_lanes = np.array(existing_lanes)
  savemat(out_name, dict(left = existing_lanes[:,0], right=existing_lanes[:, 1]))
  """
  r = np.arange(9,len(frame_data)*skip_frame,skip_frame)
  export_data = -1*np.ones((len(frame_data)*skip_frame+1,2))
  export_data[r,:] = frame_data
  left_data = -1*np.ones((len(left_frames)*skip_frame+1,2))
  left_data[r, :] = left_frames
  savemat(out_name, dict(left=left_data,right=export_data))
  """

