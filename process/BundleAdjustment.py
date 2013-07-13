import sys
from GPSReader import *
from VideoReader import *
from WGS84toENU import *
from GPSReprojection import *
from transformations import euler_matrix
from numpy import array, dot, zeros, around, divide, copy, nonzero
from cv2 import imshow, waitKey, resize, rectangle
import cv, time, scipy, scipy.optimize

## variables at global scope 
points = [ ]
I = None
M = None
records = { } 

## data gathering functions
def on_mouse(event, x, y, flags, params):
    if event == cv.CV_EVENT_LBUTTONDOWN:
        points.append((x,y))
        draw_points(points)
        print points
    if event == cv.CV_EVENT_RBUTTONDOWN:
        if (len(points)>0):
            del points[-1]
            draw_points(points)
        print points

## data drawing functions
def draw_points(points):
    I_copy = copy(I) 
    I_copy = np.minimum(I_copy, M)
    for p in points:
        rectangle(I_copy, (p[0]-2,p[1]-2), (p[0]+2,p[1]+2), (255,0,0),2)
    imshow('video', I_copy)

def redraw(I,M,points):
    I_copy = copy(I)
    I_copy = np.minimum(I_copy, M)
    for p in points:
        rectangle(I_copy, (p[0]-2,p[1]-2), (p[0]+2,p[1]+2), (255,0,0),2)
    imshow('video', I_copy)


## optimization related functions 
def l2loss(params, *args):
    records = args[0]
    cam = dict(args[1])
    print params[0], params[1], params[2]
    cam['rot_x'] = deg2rad(params[0])
    cam['rot_y'] = deg2rad(params[1])
    cam['rot_z'] = deg2rad(params[2])
    gps_dat = args[2]
    error = 0.0
    for idx in records:
        M_full = GPSMask(gps_dat[idx:idx+num_imgs_fwd,:], cam);
        M = 255-M_full[:,:,0]
        for gold in records[idx]['points']:
            xg = gold[0];
            yg = gold[1];
            minerror = 99999999.0
            nz = nonzero(M)
            for nz_idx in range(len(nz[0])):
                xi = nz[1][nz_idx]
                yi = nz[0][nz_idx]
                if (xg-xi)**2 + (yg-yi)**2 < minerror:
                    minerror = (xg-xi)**2 + (yg-yi)**2
            error += minerror
        #print idx
        I = records[idx]['I']
        points = records[idx]['points']
        redraw(I,M_full,points)
        waitKey(50)
    print error
    return error + 1000*(cam['rot_x']**2+cam['rot_y']**2+cam['rot_z']**2)




if __name__ == '__main__':
  video_filename = sys.argv[1]
  gps_filename = sys.argv[2]
  num_imgs_fwd = 500; 
  video_reader = VideoReader(video_filename)
  gps_reader = GPSReader(gps_filename)
  gps_dat = gps_reader.getNumericData()

  cam = { }
  cam['R_to_c_from_i'] = array([[-1, 0, 0], \
                         [0, 0, -1], \
                         [0, -1, 0]]);
  cam['rot_x'] = deg2rad(-0.7);
  cam['rot_y'] = deg2rad(0.3);
  cam['rot_z'] = deg2rad(0.0);

  cam['fx'] = 2221.8
  cam['fy'] = 2233.7
  cam['cu'] = 623.7
  cam['cv'] = 445.7
  cam['KK'] = array([[cam['fx'], 0.0, cam['cu']], \
                     [0.0, cam['fy'], cam['cv']], \
                     [0.0, 0.0, 1.0]]);

  cv2.namedWindow('video')
  cv.SetMouseCallback('video', on_mouse, 0)
  framenum = 0;
  lastTime = time.time()
  while True:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    if framenum % 50 != 0:
      continue
    
    M = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam); 
    R = np.minimum(M,I)
    imshow('video', R)
    key = waitKey()
    print key
    if key == ord('q'):
      break
    
    if len(points) > 0: 
        records[framenum] = { }
        records[framenum]['I'] = copy(I)
        records[framenum]['M'] = copy(M) 
        records[framenum]['points'] = points

    points = [ ]
    currentTime = time.time();
    if (currentTime - lastTime > 1):
        print framenum
        lastTime = currentTime

  optimization = scipy.optimize.fmin_l_bfgs_b(l2loss, x0=(-0.7,0.3,0.0), args=(records, cam, gps_dat), approx_grad=True, epsilon=1e-4)
  #optimization = scipy.optimize.fmin_cg(l2loss, x0=(-0.7,0.3,0.0), args=(records, cam, gps_dat), epsilon=1e-3)
  print optimization

