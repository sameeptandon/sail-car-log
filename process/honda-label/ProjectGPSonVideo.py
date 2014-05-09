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

  cam = pickle.load(open('cam_params.pickle'))[cam_num - 1]

  #framenum = 1926;
  #framenum = 29000
  framenum = 0
  lastTime = time.time()
  lastCols = [None, None]
  lastLine = [None, None, None, None]
  video_reader.setFrame(framenum)
  while True:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    #if framenum % 10 != 0:
    #  continue
    if framenum % 100 == 0:
        print framenum

    M = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, width=1); 
    I = np.minimum(M,I)
    """
    src = array([[0,0],[1280,0],[1280,960],[0,960]], float32)
    minX = 0
    minY = 0
    maxX = 1200
    maxY = 960
    dst = array([[-1000,200],[2280,200],[780,960],[500,960]], float32)
    
    #src =  array([(567, 759), (896, 756), (919, 791), (555, 793)], float32)
    src = array([(570, 737), (864, 737), (881, 761), (564, 761)], float32)
    #src = array([(520, 727), (916, 733), (950, 775), (497, 771)], float32)
    #src = array([[499,597],[972,597],[1112,661],[448,678]], float32) #good one
    #src = array([[528,560],[861,557],[1244,759],[271,773]], float32)
    #src = array([[550,524],[840,523],[1019,613],[496,612]], float32)
    ##src = array([(378, 604), (742, 601), (967, 802), (79, 784)], float32)
    ##src = array( [(445, 521), (729, 527), (1159, 819), (27, 747)], float32)
    ##src = array([(386, 521), (829, 517), (1190, 681), (92, 663)], float32)

    rx = 38
    ry = 24
    sx = 150
    sy = 100
    dst = array([[sx,sy],[sx+rx,sy],[sx+rx,sy+ry],[sx,sy+ry]],float32)

    #dst = array([[320,320],[960,320],[960,640],[320,640]], float32)
    #dst[:,0] += 960

    imsize = (320,240)
    I = resize(I, imsize)
    I[:, :5] = [255, 0, 0]
    I[:, -5:] = [255, 0, 0]
    I[-5:, :] = [0, 255, 0]
    src = src / 4;
    dst = dst ;
    #dst = array([[0,0],[1280,0],[1280,960],[0,960]], float32)
    P = getPerspectiveTransform(src,dst)
    WARP = warpPerspective(I, P, imsize);
    #WARP = resize(WARP, imsize)
    #I[0:480,:,:]=0
    I = WARP
    
    if lastCols[0] is None:
        M = 255 - resize(M, imsize)
        warped_M = np.nonzero(warpPerspective(M, P, imsize))
        col_avg = np.mean(warped_M[1])
        lastCols[0] = col_avg - 50
        lastCols[1] = col_avg + 50
    
    if lastLine[0] is None:
        lastLine[0] = 0
        lastLine[1] = lastCols[0]
        lastLine[2] = 0
        lastLine[3] = lastCols[1]


    (WARP, lastCols, lastLine) = findLanes(WARP, (imsize[1], imsize[0]), lastCols, lastLine)
    WARP = warpPerspective(WARP, P, imsize,flags=cv.CV_WARP_INVERSE_MAP);
    
    I_t = np.zeros((imsize[1], imsize[0], 3))
    I_t[239/2, :, 0] = 255
    I_t = warpPerspective(I_t, P, imsize, flags=cv.CV_WARP_INVERSE_MAP)
    I[WARP[:,:,0] > 0, 0] = 0
    I[WARP[:,:,0] > 0, 1] = 0
    I[WARP[:,:,0] > 0, 2] = 255
    #I[I_t[:, :, 0] > 0, 0] = 255
    #I[I_t[:, :, 0] > 0, 1] = 0
    #I[I_t[:, :, 0] > 0, 2] = 0

    if lastCols[0] is not None and lastCols[1] is not None:
        I[:,lastCols[0],:] = 0
        I[:,lastCols[1],:] = 0
        I[:,(lastCols[0]+lastCols[1])/2,:] = 0
    """
    #I = warpPerspective(I, P, imsize, flags=cv.CV_WARP_INVERSE_MAP)
    I = resize(I, (640, 480))
    imshow('video', I )
    key = (waitKey(2) & 255)
    if key == ord('q'):
      break;
    currentTime = time.time();
    if (currentTime - lastTime > 1):
        
        lastTime = currentTime

