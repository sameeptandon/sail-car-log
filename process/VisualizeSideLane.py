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
from copy import deepcopy
import matplotlib.pylab as pp
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
  num_imgs_fwd = 100;
  video_reader = VideoReader(video_filename)
  gps_reader = GPSReader(gps_filename)
  gps_dat = gps_reader.getNumericData()
  gps_latlong = gps_dat[:, 1:3]
  #latlong = np.array([[37.333524, -122.074422],[37.333520, -122.074504],[37.333517, -122.074614],[37.333515, -122.074701],[37.333514, -122.074776],[37.333514, -122.074859],[37.333515, -122.074975],[37.333518, -122.075099],[37.333524, -122.075249],[37.333530, -122.075381]])

  latlong = np.loadtxt('markings')
  cam = pickle.load(open('cam_params.pickle'))[cam_num - 1]

  framenum = 0
  lastTime = time.time()
  lastCols = [None, None]
  lastLine = [None, None, None, None]
  video_reader.setFrame(framenum)
  lane_start=9450
  while True:
    lane_forward=framenum*6
    lane_pts = np.empty([851,10])
    lane_pts[:,1:3]=latlong[lane_start+lane_forward:lane_start+851+lane_forward,:]
    for ll in xrange(851):
      closest_idx = np.argmin(np.sum((gps_latlong-latlong[lane_start+lane_forward+ll,:])**2, axis=1))
      lane_pts[ll,3] = gps_dat[closest_idx,3]

    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    #if framenum % 10 != 0:
    #  continue
    if framenum % 100 == 0:
        print framenum

    aa=np.concatenate((np.array([gps_dat[framenum,:]]), lane_pts),axis=0)
    L = GPSMask(np.concatenate((np.array([gps_dat[framenum,:]]), lane_pts),axis=0), cam, width=3); 
    M = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, width=3); 
    I = np.minimum(M,I)
    I = np.minimum(L,I)
    #I = warpPerspective(I, P, imsize, flags=cv.CV_WARP_INVERSE_MAP)
    I = resize(I, (640, 480))
    #origname = '/afs/cs.stanford.edu/u/twangcat/scratch/sail-car-log/process/side_lane_imgs/%d.png'%(int(framenum))
    #pp.imsave(origname, I)
    imshow('video', I )
    key = (waitKey(2) & 255)
    if key == ord('q'):
      break;
    currentTime = time.time();
    if (currentTime - lastTime > 1):
        
        lastTime = currentTime


