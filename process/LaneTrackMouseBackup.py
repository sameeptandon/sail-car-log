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
from scipy.io import savemat,loadmat
from generate_lane_labels import * 
import sys

pause_labeler = False
mouse_position = None
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
        #print mouse_click


if __name__ == '__main__':
  video_filename = sys.argv[2]
  if sys.argv[1]=='Left':
    Left=True
  elif sys.argv[1]=='Right':
    Left=False
  else:
    sys.exit("first argument must be Left or Right!")
  path, vfname = os.path.split(video_filename)
  vidname = vfname.split('.')[0]
  cam_num = int(vidname[-1])
  gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
  if Left:
    out_name = sys.argv[3]
  else:
    in_name = sys.argv[3]
    oldmat = loadmat(in_name)
    oldleft=oldmat['left']
    out_name = sys.argv[4]
  display = True
  if '--quiet' in sys.argv:
      display = False


  cv2.namedWindow('video')
  cv.SetMouseCallback('video', on_mouse, 0)
  num_imgs_fwd = 200; 
  video_reader = VideoReader(video_filename, num_splits=10)
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
  skip_frame=5
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
      video_reader.setFrame(framenum)
      (success, I) = video_reader.getFrame(framenum)
      if success == False:
        break
      prevI = np.copy(I)
    else:
      I = np.copy(prevI)

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
        imshow('video', I_WARP)
        imshow('original', Orig_I)
        key = (waitKey(frameWaitTime) & 255)
    frameWaitTime = 1#max(int(frameWaitTime / 1.5), 2)
    if framenum==0: 
      time.sleep(2)
    #Orig_I[unwarped_point[1]/4-1:unwarped_point[1]/4+1, unwarped_point[0]/4-1:unwarped_point[0]/4+1,:]=[255,0,0]
    #imshow('test', Orig_I)
    if mouse_click is not None:
      point = mouse_click
      x = point[0]
      y=np.nonzero(M_WARP[:,x])[0][-1]
      #G_Local = G_WARP[y-20:y+1, x-15:x+15]
      #maxidx = np.unravel_index(np.argmax(G_Local), G_Local.shape)
      #x=x-15+maxidx[1]
      #y=y-20+maxidx[0]
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

    if not paused or framenum==0:
      framenum += skip_frame

  r = np.arange(skip_frame-1,len(frame_data)*skip_frame,skip_frame)
  export_data = -1*np.ones((len(frame_data)*skip_frame+1,2))
  export_data[r,:] = frame_data
  if Left:
    savemat(out_name, dict(left=export_data))
  else:
    if len(export_data)>oldleft.shape[0]:
      export_data = export_data[0:oldleft.shape[0],:]
    if len(export_data)<oldleft.shape[0]:
      oldleft = oldleft[0:len(export_data),:] 
    savemat(out_name, dict(left = oldleft, right=export_data))

