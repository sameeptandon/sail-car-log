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
from generate_lane_labels import * 

def on_mouse(event, x, y, flags, params):
    if event == cv.CV_EVENT_LBUTTONDOWN:
        print 'click: ', (x,y)
        print 'color: ', I[y,x,:]


if __name__ == '__main__':
  video_filename = sys.argv[1]
  path, vfname = os.path.split(video_filename)
  vidname = vfname.split('.')[0]
  cam_num = int(vidname[-1])
  gps_filename = path + '/' + vidname[0:-1] + '_gps.out' 
  #gps_filename = sys.argv[2]

  cv2.namedWindow('video')
  cv.SetMouseCallback('video', on_mouse, 0)
  num_imgs_fwd = 1250; 
  video_reader = VideoReader(video_filename)
  gps_reader = GPSReader(gps_filename)
  gps_dat = gps_reader.getNumericData()

  cam = { }
  cam['R_to_c_from_i'] = array([[-1, 0, 0], \
                         [0, 0, -1], \
                         [0, -1, 0]]);

  if cam_num == 1:
    cam['rot_x'] = deg2rad(-0.8); # better cam 1
    cam['rot_y'] = deg2rad(-0.5);
    cam['rot_z'] = deg2rad(-0.005);
    cam['t_x'] = -0.5;
    cam['t_y'] = 1.1;
    cam['t_z'] = 0.0;
  elif cam_num == 2: 
    cam['rot_x'] = deg2rad(-0.61); # better cam 2 
    cam['rot_y'] = deg2rad(0.2);
    cam['rot_z'] = deg2rad(0.0);
    cam['t_x'] = 0.5;
    cam['t_y'] = 1.1;
    cam['t_z'] = 0.0;

  """
  cam['rot_x'] = deg2rad(-0.66); # cam 1 experimental
  cam['rot_y'] = deg2rad(-0.71);
  cam['rot_z'] = deg2rad(-0.005);
  """
  """
  cam['rot_x'] = deg2rad(-0.62); # cam 2 experimental
  cam['rot_y'] = deg2rad(0.5);
  cam['rot_z'] = deg2rad(0.027);
  """

  cam['fx'] = 2221.8
  cam['fy'] = 2233.7
  cam['cu'] = 623.7
  cam['cv'] = 445.7
  cam['KK'] = array([[cam['fx'], 0.0, cam['cu']], \
                     [0.0, cam['fy'], cam['cv']], \
                     [0.0, 0.0, 1.0]]);

  #framenum = 1926;
  framenum = 0
  lastTime = time.time()
  lastCols = [None, None]
  video_reader.setFrame(framenum)
  while True:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    #if framenum % 10 != 0:
    #  continue

    M = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, width=1); 
    I = np.minimum(M,I)
    """
    src = array([[0,0],[1280,0],[1280,960],[0,960]], float32)
    minX = 0
    minY = 0
    maxX = 1200
    maxY = 960
    dst = array([[-1000,200],[2280,200],[780,960],[500,960]], float32)
    """
    src = array([[499,597],[972,597],[1112,661],[448,678]], float32) #good one
    #src = array([[528,560],[861,557],[1244,759],[271,773]], float32)
    #src = array([[550,524],[840,523],[1019,613],[496,612]], float32)
    ##src = array([(378, 604), (742, 601), (967, 802), (79, 784)], float32)
    ##src = array( [(445, 521), (729, 527), (1159, 819), (27, 747)], float32)
    ##src = array([(386, 521), (829, 517), (1190, 681), (92, 663)], float32)

    dst = array([[320,320],[960,320],[960,640],[320,640]], float32)

    imsize = (320,240)
    I = resize(I, imsize)
    src = src / 4;
    dst = dst / 4;
    #dst = array([[0,0],[1280,0],[1280,960],[0,960]], float32)
    P = getPerspectiveTransform(src,dst)
    WARP = warpPerspective(I, P, imsize);
    #WARP = resize(WARP, imsize)
    #I[0:480,:,:]=0
    I = WARP
    (WARP, lastCols) = findLanes(WARP, (imsize[1], imsize[0]), lastCols)
    print 'lastCols = ', lastCols
    #WARP = warpPerspective(WARP, P, imsize,flags=cv.CV_WARP_INVERSE_MAP);
    
    I[WARP[:,:,0] > 0, 0] = 0
    I[WARP[:,:,0] > 0, 2] = 0
    I[WARP[:,:,0] > 0, 1] = 255
    if lastCols[0] is not None and lastCols[1] is not None:
        I[:,lastCols[0],:] = 0
        I[:,lastCols[1],:] = 0
        I[:,(lastCols[0]+lastCols[1])/2,:] = 0
    imshow('video', I )
    key = (waitKey(4) & 255)
    if key == ord('q'):
      break;
    currentTime = time.time();
    if (currentTime - lastTime > 1):
        print framenum
        lastTime = currentTime

