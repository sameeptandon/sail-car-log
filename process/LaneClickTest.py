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
    if event == cv.CV_EVENT_LBUTTONDOWN:
        mouse_click = (x, y)
        last_click_time = time.time() 
        print mouse_click


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

  while True:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break

    #if framenum % skip_frame != 0:
    #  continue

    if framenum % 150 == 0:
        r = np.arange(9,len(frame_data)*skip_frame,skip_frame)
        export_data = -1*np.ones((len(frame_data)*skip_frame+1,2))
        export_data[r,:] = frame_data
        left_data = -1*np.ones((len(left_frames)*skip_frame+1,2))
        left_data[r, :] = left_frames
        savemat(out_name, dict(left=left_data,right=export_data))

    I = resize(I, imsize)
    I_WARP = warpPerspective(I, P, imsize);
    
    if lastCols[0] is None:
        M = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, width=1); 
        M = 255 - resize(M, imsize)
        warped_M = np.nonzero(warpPerspective(M, P, imsize))
        col_avg = np.mean(warped_M[1])
        lastCols[0] = col_avg - default_offset
        lastCols[1] = col_avg + default_offset

    if mouse_click is not None:
        col_avg = mouse_click[0] / 2
        dist = min(0.95 * (lastCols[1] - lastCols[0]) / 2, col_avg)
        delta_time = last_click_time - lastClickProcessed
        print 'delta time: ', delta_time
        dist *= min(delta_time,1)
        lastCols[0] = col_avg - dist
        lastCols[1] = col_avg + dist
        print dist
        print lastCols
        mouse_click = None
        lastClickProcessed = last_click_time
        frameWaitTime = min(500, frameWaitTime*10)
    
    if lastLine[0] is None:
        lastLine[0] = 0
        lastLine[1] = lastCols[0]
        lastLine[2] = 0
        lastLine[3]= lastCols[1]

   
    #I = np.copy(WARP)
    (O, lastCols,asdf2) = findLanesConvolution(I, (imsize[1], imsize[0]), lastCols, lastLine, P=P, frame_rate=skip_frame)
    (O, lastCols,asdf2) = findLanes(O, (imsize[1], imsize[0]), lastCols, lastLine, frame_rate=skip_frame)
    line_img = np.zeros((imsize[1], imsize[0]))
    line_img[20:200,:] = 1

    #I_WARP = O
    I_WARP[O[:,:,0] > 0, :] = [0, 0, 255]
    I[line_img > 0, :] = [255, 0, 0]

    good_img = np.copy(O)
    good_img[line_img == 0, : ] = 0
    left_img = np.copy(good_img)
    right_img = np.copy(good_img)
    left_img[:,(lastCols[0]+lastCols[1])/2:,:] = 0
    right_img[:,:(lastCols[0]+lastCols[1])/2,:] = 0
    I_WARP[good_img > 0]  = 0
    candidates = np.nonzero(good_img)
    left_candidates = np.nonzero(left_img)
    candidates = np.nonzero(right_img)

    if lastCols[0] is not None and lastCols[1] is not None:
        I_WARP[:,lastCols[0],:] = 0
        I_WARP[:,lastCols[1],:] = 0
        I_WARP[:,(lastCols[0]+lastCols[1])/2,:] = 0

    #I_WARP = cv2.warpPerspective(I_WARP, P, imsize, flags=cv.CV_WARP_INVERSE_MAP) 
    I = resize(I_WARP, (640,480))
    key = ''
    if display:
        imshow('video',I)
        key = (waitKey(frameWaitTime) & 255)
    frameWaitTime = max(int(frameWaitTime / 1.5), 5)

    if left_candidates[0].size > 0:
        chosen_idx = np.argmax(left_candidates[0])
        point = (left_candidates[1][chosen_idx], left_candidates[0][chosen_idx])
        point_img = np.zeros((imsize[1], imsize[0]), dtype=np.uint8)
        point_img[point[1]-3:point[1]+3, point[0]-3:point[0]+3] = 355
        I_WARP[point_img > 0] = [255, 255, 0]
        unwarped_img = warpPerspective(point_img, P, imsize, flags=cv.CV_WARP_INVERSE_MAP)
        unwarped_point = np.nonzero(unwarped_img)
        if (len(unwarped_point[0]) > 0):
            unwarped_point = (unwarped_point[1][0] * 4 + 2, unwarped_point[0][0] * 4 + 2)
            left_frames.append(unwarped_point)
        else:
            left_frames.append((-1, -1))
    else:
        left_frames.append((-1, -1))

    if candidates[0].size > 0:
        chosen_idx = np.argmax(candidates[0])
        point = (candidates[1][chosen_idx], candidates[0][chosen_idx])
        point_img = np.zeros((imsize[1],imsize[0]));
        point_img[point[1]-3:point[1]+3, point[0]-3:point[0]+3] = 1
        unwarped_img = warpPerspective(point_img, P, imsize, flags=cv.CV_WARP_INVERSE_MAP)
        unwarped_point = np.nonzero(unwarped_img) 
        if (len(unwarped_point[0]) > 0): 
            unwarped_point = (unwarped_point[1][0]*4 + 2, unwarped_point[0][0]*4 + 2)
            frame_data.append(unwarped_point)
        else:
            frame_data.append((-1,-1))
    else:
        frame_data.append((-1,-1))

    if key == ord('q'):
      break;
    currentTime = time.time();
    if (currentTime - lastTime > 10):
        lastTime = currentTime
        print framenum

  r = np.arange(9,len(frame_data)*skip_frame,skip_frame)
  export_data = -1*np.ones((len(frame_data)*skip_frame+1,2))
  export_data[r,:] = frame_data
  left_data = -1*np.ones((len(left_frames)*skip_frame+1,2))
  left_data[r, :] = left_frames
  savemat(out_name, dict(left=left_data,right=export_data))

