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
frame_data = [ ] 

def on_mouse(event, x, y, flags, params):
    global pause_labeler
    global mouse_position
    if event == cv.CV_EVENT_LBUTTONDOWN:
        print 'click: ', (x,y)
        print 'color: ', I[y,x,:]
    if event == cv.CV_EVENT_MOUSEMOVE:
        print 'move: ', (x,y)
        pause_labeler = True
        mouse_position = (x,y)


if __name__ == '__main__':
  video_filename = sys.argv[1]
  path, vfname = os.path.split(video_filename)
  vidname = vfname.split('.')[0]
  cam_num = int(vidname[-1])
  gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
  out_name = sys.argv[2]

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
  #framenum = 27000
  framenum = 0
  lastTime = time.time()
  lastCols = [None, None]
  lastLine = [None, None, None, None]
  video_reader.setFrame(framenum)

  skip_frame = 30 


  while True:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    if framenum % skip_frame != 0:
      continue

    M = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, width=1); 
    I = np.minimum(M,I)
    
    
    src = array([[499,597],[972,597],[1112,661],[448,678]], float32) #good one
    dst = array([[320,320],[960,320],[960,640],[320,640]], float32)

    imsize = (320,240)
    I = resize(I, imsize)
    src = src / 4;
    dst = dst / 4;
    P = getPerspectiveTransform(src,dst)
    WARP = warpPerspective(I, P, imsize);
    I = np.copy(WARP)
    
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
        lastLine[3]= lastCols[1]
    
    (WARP, asdf1,asdf2) = findLanes(WARP, (imsize[1], imsize[0]), lastCols, lastLine, responseOnlyNearLastCols=False)
    #I = WARP

    line_img = np.zeros((imsize[1], imsize[0]))
    line_img[158:162,:] = 1

    I[WARP[:,:,0] > 0, :] = [0, 0, 255]
    #I[line_img > 0, :] = [255, 0, 0]

    good_img = np.copy(WARP)
    good_img[line_img == 0, : ] = 0
    #good_img[:,(lastCols[0]+lastCols[1])/2:,:] = 0
    good_img[:,:(lastCols[0]+lastCols[1])/2,:] = 0
    I[good_img > 0]  = 0
    candidates = np.nonzero(good_img)

    if lastCols[0] is not None and lastCols[1] is not None:
        I[:,lastCols[0],:] = 0
        I[:,lastCols[1],:] = 0
        I[:,(lastCols[0]+lastCols[1])/2,:] = 0

    I = resize(I, (1280,960))
    imshow('video',I / 255.0 )
    key = (waitKey(50) & 255)
    if pause_labeler == True: 
        key = waitKey(5)
        pause_labeler = False

    if candidates[0].size > 0:
        chosen_idx = np.argmin(candidates[1])
        point = (candidates[1][chosen_idx], candidates[0][chosen_idx])
        point_img = np.zeros((imsize[1],imsize[0]));
        point_img[point[1]-2:point[1]+2, point[0]-2:point[0]+2] = 1
        unwarped_img = warpPerspective(point_img, P, imsize, flags=cv.CV_WARP_INVERSE_MAP)
        unwarped_point = np.nonzero(unwarped_img) 
        if (len(unwarped_point[0]) > 0): 
            unwarped_point = (unwarped_point[1][0]*4, unwarped_point[0][0]*4)
            frame_data.append(unwarped_point)
        else:
            frame_data.append((-1,-1))
    else:
        frame_data.append((-1,-1))


    if mouse_position is not None:
        #lastCols[0] = mouse_position[0] / 4
        lastCols[1] = mouse_position[0] / 4
        lastCols[0] = lastCols[1] - 150
    
    if key == ord('q'):
      break;
    currentTime = time.time();
    if (currentTime - lastTime > 30):
        lastTime = currentTime
        print framenum

  r = np.arange(0,len(frame_data)*skip_frame,skip_frame)
  export_data = -1*np.ones((len(frame_data)*skip_frame+1,2))
  export_data[r,:] = frame_data
  savemat(out_name, dict(left=export_data,right=export_data))

