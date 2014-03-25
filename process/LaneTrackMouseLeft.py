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
from generate_lane_labels import * 


pause_labeler = False
mouse_position = None
left_frames = []
frame_data = []
mouse_click = None
last_click_time = 0
frameWaitTime = 100

def on_mouse(event, x, y, flags, params):
    global pause_labeler
    global mouse_position
    global mouse_click
    global last_click_time
    if event == cv.CV_EVENT_MOUSEMOVE:
        mouse_click = (x, y)
        last_click_time = time.time() 


if __name__ == '__main__':
  video_filename = sys.argv[1]
  path, vfname = os.path.split(video_filename)
  vidname = vfname.split('.')[0]
  cam_num = int(vidname[-1])
  gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
  out_name = sys.argv[2]
  display = True
  if '--quiet' in sys.argv:
      display = False


  cv2.namedWindow('video')
  cv.SetMouseCallback('video', on_mouse, 0)
  num_imgs_fwd = 200; 
  video_reader = VideoReader(video_filename, num_splits=1)
  gps_reader = GPSReader(gps_filename)
  gps_dat = gps_reader.getNumericData()

  cam = pickle.load(open('cam_params.pickle', 'rb'))[cam_num - 1]

  framenum = 0
  lastTime = time.time()
  lastClickProcessed = time.time()
  lastCols = [None, None]
  lastLine = [None, None, None, None]
  x=None
  y=None
  video_reader.setFrame(framenum)

  skip_frame = 10
  default_offset = 60

  src = array([(570, 737), (864, 737), (881, 761), (564, 761)], float32) / 4
  rx = 33
  ry = 21
  sx = 150
  sy = 120
  dst = array([[sx,sy],[sx+rx,sy],[sx+rx,sy+ry],[sx,sy+ry]],float32)
  #src = array([[499,597],[972,597],[1112,661],[448,678]], float32) / 4#good one
  #dst = array([[320,320],[960,320],[960,640],[320,640]], float32) / 4
  imsize = (320,240)
  P = getPerspectiveTransform(src,dst)

  paused = False

  while True:
    if not paused or framenum==0:
      framenum = framenum + 1
      
      (success, I) = video_reader.getNextFrame()
      if success == False:
        break
      prevI = np.copy(I)
    else:
      I = np.copy(prevI)
    #if framenum % skip_frame != 0:
    #  continue

    #if framenum % 150 == 0:
    if framenum % 2 == 0:
        r = np.arange(9,len(frame_data)*skip_frame,skip_frame)
        export_data = -1*np.ones((len(frame_data)*skip_frame+1,2))
        export_data[r,:] = frame_data
        savemat(out_name, dict(left=export_data))

    I = resize(I, imsize)
    Orig_I = np.copy(I)
    Orig_M = np.ones((imsize[1], imsize[0]), dtype=np.uint8)*255
    I_WARP = warpPerspective(I, P, imsize);
    G_WARP = np.dot(I_WARP[...,:3], [0.299, 0.587, 0.144])
    M_WARP = warpPerspective(Orig_M, P, imsize);
    
    if x is not None and y is not None:
      I_WARP[y-3:y+3, x-3:x+3,0]=255
    #I_WARP = cv2.warpPerspective(I_WARP, P, imsize, flags=cv.CV_WARP_INVERSE_MAP) 
    key = ''
    if display:
        imshow('video', resize(I_WARP, (640,480)))
        key = (waitKey(frameWaitTime) & 255)
    frameWaitTime = max(int(frameWaitTime / 1.5), 5)
    if framenum==1: 
      time.sleep(2)
    #Orig_I[unwarped_point[1]/4-1:unwarped_point[1]/4+1, unwarped_point[0]/4-1:unwarped_point[0]/4+1,:]=[255,0,0]
    #imshow('test', Orig_I)
    if mouse_click is not None:
      point = mouse_click
      x = point[0]/2
      y=np.nonzero(M_WARP[:,x])[0][-1]
      #G_Local = G_WARP[y-5:y+1, x-5:x+5]
      #maxidx = np.unravel_index(np.argmax(G_Local), G_Local.shape)
      #x=x-5+maxidx[1]
      #y=y-5+maxidx[0]
      point_img = np.zeros((imsize[1],imsize[0]));                                                                
      point_img[y-3:y+3, x-3:x+3] = 1
      unwarped_img = warpPerspective(point_img, P, imsize, flags=cv.CV_WARP_INVERSE_MAP)
      unwarped_point = np.nonzero(unwarped_img) 
      if (len(unwarped_point[0]) > 0): 
          unwarped_point = (unwarped_point[1][0]*4 + 2, unwarped_point[0][0]*4 + 2)
          frame_data.append(unwarped_point)
      else:
          frame_data.append((-1,-1))

    if key == ord('q'):
      break;
    if key == ord('p'):
      paused = not paused
    currentTime = time.time();
    if (currentTime - lastTime > 10):
        lastTime = currentTime
        print framenum

  r = np.arange(9,len(frame_data)*skip_frame,skip_frame)
  export_data = -1*np.ones((len(frame_data)*skip_frame+1,2))
  export_data[r,:] = frame_data
  savemat(out_name, dict(left=export_data))

