import sys, os
from GPSReader import *
from VideoReader import *
from WGS84toENU import *
from GPSReprojection import *
from transformations import euler_matrix
from numpy import array, dot, zeros, around, divide, nonzero, float32, maximum, copy
import numpy as np
from cv2 import imshow, waitKey, resize, warpPerspective, getPerspectiveTransform, transpose, Canny, namedWindow, rectangle
import cv
import time
from generate_lane_labels import * 
import pickle

points = [ ]
I = None

def closest_point(target,pts):
    closest = None
    min_closest= 9999999999
    min_idx = 0
    for i in range(len(pts)):
        score = point_distance(pts[i], target)
        if score < min_closest:
            closest = pts[i]
            min_closest = score
            min_idx = i 
    return (closest, min_idx)
 
def point_distance(p1,p2):
    return (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2

def on_mouse(event, x, y, flags, params):
    if event == cv.CV_EVENT_LBUTTONDOWN:
        print 'click: ', (x,y)
        if len(points) < 4:
            points.append((x,y))
        elif len(points) == 4:
            (pt, idx) = closest_point((x,y), points)
            points[idx] = (x,y)
            show_perspective()
        redraw()

    if event == cv.CV_EVENT_RBUTTONDOWN:
        print 'right button'
        if len(points) == 4:
            print 'calling perspective'
            show_perspective()

def redraw():
    I_copy = copy(I)
    for p in points:
        rectangle(I_copy, (p[0]-2,p[1]-2), (p[0]+2,p[1]+2), (255,0,0),2)
    imshow('video', I_copy)

def show_perspective():
    global points
    src = array(points, float32)
    print 'points = ', points
    dxmin = 480
    dxmax = 1280-dxmin
    dymin = 320
    dymax = 960-dymin
    #dst = array([[dxmin,dymin],[dxmax,dymin],[dxmax,dymax],[dxmin,dymax]], float32)
    dst = array([[0,0],[1280,0],[1280,960],[0,960]], float32)
    P = getPerspectiveTransform(src,dst)
    WARP = warpPerspective(I, P, (1280,960));
    imshow('perspective', WARP)
    #WARP = resize(WARP, (320,240))
    srcs = [ ]
    srcs.append(array([(0, 90), (1280, 0), (1280, 870), (0, 960)], float32))
    srcs.append(array([(0, 0), (1280, 90), (1280, 960), (0, 870)], float32))
    srcs.append(array([(130, 0), (1280, 0), (1150, 960), (0, 960)], float32))
    srcs.append(array([(0, 0), (1150, 0), (1280, 960), (90, 960)], float32))
    srcs.append(array([(0,0), (1280-90,90),(1280,960-90),(0,960)], float32))
    srcs.append(array([(0,90), (1280-90,90),(1280,960-90),(90,960-90)], float32))
    P_transforms = [ ]
    P_transforms = pickle.load(open('perspective_transforms.pickle'))
    for s in range(len(srcs)):
        #P = getPerspectiveTransform(srcs[s], dst);
        #P_transforms.append(P)
        WARP = warpPerspective(I, P_transforms[s], (1280,960))
        imshow('default_'+str(s), WARP) 
    #pickle.dump(P_transforms, open('perspective_transforms.pickle','w'))

if __name__ == '__main__':
  video_filename = sys.argv[1]
  path, vfname = os.path.split(video_filename)
  vidname = vfname.split('.')[0]
  cam_num = int(vidname[-1])
  gps_filename = path + '/' + vidname[0:-1] + '_gps.out' 

  cv2.namedWindow('video')
  cv.SetMouseCallback('video', on_mouse, 0)
  num_imgs_fwd = 1250; 
  video_reader = VideoReader(video_filename, num_splits=1)
  gps_reader = GPSReader(gps_filename)
  gps_dat = gps_reader.getNumericData()

  cam = { }
  cam['R_to_c_from_i'] = array([[-1, 0, 0], \
                         [0, 0, -1], \
                         [0, -1, 0]]);

  framenum = 0
  lastTime = time.time()
  lastCols = [None, None]
  while True:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    if framenum % 10 != 0:
      continue

    imshow('video', I)
    if len(points) == 4:
        redraw()
        show_perspective()
    key = waitKey() & 255
    print key
    print chr(key)
    if key == ord('q'):
      break;
    currentTime = time.time();
    if (currentTime - lastTime > 1):
        print framenum
        lastTime = currentTime

