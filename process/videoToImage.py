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
from generate_lane_labels import * 
import matplotlib.pylab as pp
import pickle
from scipy.io import savemat
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
  num_imgs_fwd = 200;
  video_reader = VideoReader(video_filename)
  gps_reader = GPSReader(gps_filename)
  gps_dat = gps_reader.getNumericData()
  print gps_dat.shape

  cam = pickle.load(open('cam_params.pickle'))[cam_num - 1]
  framenum = 5999
  lastTime = time.time()
  lastCols = [None, None]
  lastLine = [None, None, None, None]
  video_reader.setFrame(framenum)
  while framenum<7500:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    #if framenum % 10 != 0:
    #  continue
    if framenum % 100 == 0:
        print framenum
    M = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, width=1);
    if framenum==6000:
      #Coord = GPSColumns(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, gps_dat[framenum])
      Coord3d = GPSPos(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, gps_dat[framenum])
    else:
      #Coord = np.concatenate((Coord,GPSColumns(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, gps_dat[framenum])), axis=0)
      Coord3d = np.concatenate((Coord3d,GPSPos(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, gps_dat[framenum])), axis=0)
    I = np.minimum(M,I)

    #I = warpPerspective(I, P, imsize, flags=cv.CV_WARP_INVERSE_MAP)
    #I = resize(I, (640, 480))
    #origname = '/afs/cs/group/brain/scailsave/driving_backup/tempVideo/%d.png'%(int(framenum-6000))
    #pp.imsave(origname, I)
    #imshow('video', I )
    #key = (waitKey(2) & 255)
    #if key == ord('q'):
    #  break;
    #currentTime = time.time();
    #if (currentTime - lastTime > 1):
    #    
    #    lastTime = currentTime
  #print Coord.shape

  savemat('/afs/cs/group/brain/scailsave/driving_backup/tempVideo/coord3d.mat', {'coord3d': Coord3d})
