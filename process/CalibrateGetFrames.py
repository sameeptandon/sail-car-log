import cv2, sys, os
import numpy as np

from VideoReader import *

patternShape = (7,10)

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

  firstTime = True
  ct = 1;
  fileList = open(outdir+'/filelist.txt', 'w');

  while True:
    framenum = framenum + 1;
    (success, I_right) = video_reader_cam_1.getNextFrame()
    (success, I_left) = video_reader_cam_2.getNextFrame()
    if success == False:
      break
    if framenum % 5 != 0:
        continue
    I_right_orig = I_right.copy()
    I_left_orig = I_left.copy()

    I_right = cv2.cvtColor(I_right, cv2.COLOR_BGR2GRAY)
    I_left = cv2.cvtColor(I_left, cv2.COLOR_BGR2GRAY)
  
    """
    flags = cv2.CALIB_CB_FAST_CHECK
    ret_right, right_corners = cv2.findChessboardCorners(I_right, patternShape, right_corners, flags=flags)
    ret_left, left_corners = cv2.findChessboardCorners(I_left, patternShape, left_corners, flags=flags)

    if ret_right == False or ret_left == False:
        continue

    cv2.cornerSubPix(I_right, right_corners, (10,10), (-1,-1), criteria)
    cv2.cornerSubPix(I_left, left_corners, (10,10), (-1,-1), criteria)
    I_left_cb = I_left_orig.copy()
    I_right_cb = I_right_orig.copy()
    cv2.drawChessboardCorners(I_right_cb, patternShape, right_corners, ret_right)
    cv2.drawChessboardCorners(I_left_cb, patternShape, left_corners, ret_left)
    cv2.imshow('right', cv2.resize(I_right_cb, (640,480)))
    cv2.imshow('left', cv2.resize(I_left_cb, (640,480)))
    key = chr((cv2.waitKey(5) & 255))
    if key == 'q':
        break
    #if key == 'k':
    left_imgpoints.append(left_corners.reshape(-1,2))
    right_imgpoints.append(right_corners.reshape(-1,2))
    """
    cv2.imwrite(outdir+'/left'+str(ct)+'.ppm', I_left_orig)
    cv2.imwrite(outdir+'/right'+str(ct)+'.ppm', I_right_orig)
    fileList.write('left'+str(ct)+'.ppm\n')
    fileList.write('right'+str(ct)+'.ppm\n')
    ct += 1
    #left_frames.append(np.copy(I_left))
    #right_frames.append(np.copy(I_right))


"""
import pickle
#pickle.dump(left_frames, open(outdir + '/' + outname + '_lf.pickle', 'wb'))
#pickle.dump(right_frames, open(outdir + '/' + outname + '_rf.pickle', 'wb'))
pickle.dump(left_imgpoints, open(outdir + '/' + outname + '_lpt.pickle', 'wb'))
pickle.dump(right_imgpoints, open(outdir + '/'+  outname + '_rpt.pickle', 'wb'))
"""
