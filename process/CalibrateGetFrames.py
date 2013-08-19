import cv2, sys, os
import numpy as np

from VideoReader import *

patternShape = (8,12)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

if __name__ == '__main__':
  video_filename = sys.argv[1]
  path, vfname = os.path.split(video_filename)
  vfname_2 = vfname.replace('1','2')
  outdir = sys.argv[2]
  outname = sys.argv[3]

  cv2.namedWindow('video')
  video_reader_cam_1 = VideoReader(video_filename)
  video_reader_cam_2 = VideoReader(path + '/' + vfname_2)
  
  framenum = 0
  right_corners = None
  left_corners = None
  right_imgpoints = [ ] 
  left_imgpoints = [ ] 
  right_frames = [ ] 
  left_frames = [ ] 

  while True:
    framenum = framenum + 1;
    (success, I_right) = video_reader_cam_1.getNextFrame()
    (success, I_left) = video_reader_cam_2.getNextFrame()
    if success == False:
      break
    if framenum % 50 != 0:
        continue
    I_right = cv2.cvtColor(I_right, cv2.COLOR_BGR2GRAY)
    I_left = cv2.cvtColor(I_left, cv2.COLOR_BGR2GRAY)
    
    flags = cv2.CALIB_CB_FAST_CHECK
    ret_right, right_corners = cv2.findChessboardCorners(I_right, patternShape, right_corners, flags=flags)
    ret_left, left_corners = cv2.findChessboardCorners(I_left, patternShape, left_corners, flags=flags)
    if ret_right == True and ret_left == True:
        cv2.cornerSubPix(I_right, right_corners, (4,4), (-1,-1), criteria)
        cv2.cornerSubPix(I_left, left_corners, (4,4), (-1,-1), criteria)
        left_imgpoints.append(left_corners.reshape(-1,2))
        right_imgpoints.append(right_corners.reshape(-1,2))
        #left_frames.append(np.copy(I_left))
        #right_frames.append(np.copy(I_right))
        cv2.drawChessboardCorners(I_right, patternShape, right_corners, ret_right)
        cv2.drawChessboardCorners(I_left, patternShape, left_corners, ret_left)

    cv2.imshow('right', I_right)
    cv2.imshow('left', I_left)
    key = chr((cv2.waitKey(5) & 255))
    if key == 'q':
        break

import pickle
#pickle.dump(left_frames, open(outdir + '/' + outname + '_lf.pickle', 'wb'))
#pickle.dump(right_frames, open(outdir + '/' + outname + '_rf.pickle', 'wb'))
pickle.dump(left_imgpoints, open(outdir + '/' + outname + '_lpt.pickle', 'wb'))
pickle.dump(right_imgpoints, open(outdir + '/'+  outname + '_rpt.pickle', 'wb'))
