import sys
from GPSReader import *
from VideoReader import *
from WGS84toENU import *
from transformations import euler_matrix
from numpy import array, dot, zeros, around, divide
from cv2 import imshow, waitKey

if __name__ == '__main__':
  video_filename = sys.argv[1]
  gps_filename = sys.argv[2]

  num_imgs_fwd = 500; 

  video_reader = VideoReader(video_filename)
  gps_reader = GPSReader(gps_filename)
  gps_dat = gps_reader.getNumericData()
  
  R_to_c_from_i = array([[-1, 0, 0], \
                         [0, 0, -1], \
                         [0, -1, 0]]);
  R_camera_pitch = euler_matrix(-deg2rad(0.0), deg2rad(0), deg2rad(0.0), 'sxyz')[0:3,0:3]
  R_to_c_from_i = dot(R_camera_pitch, R_to_c_from_i) 

  cam = { }
  cam['fx'] = 2221.8
  cam['fy'] = 2233.7
  cam['cu'] = 623.7
  cam['cv'] = 445.7
  #cam['f'] = 2271.3;
  #cam['cu'] = 622.0338;
  #cam['cv'] = 419.4885;
  #cam['f'] = 2250.0;
  #cam['cu'] = 640.0;
  #cam['cv'] = 480.0;
  cam['KK'] = array([[cam['fx'], 0.0, cam['cu']], \
                     [0.0, cam['fy'], cam['cv']], \
                     [0.0, 0.0, 1.0]]);

  framenum = 0;
  while True:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    if framenum % 5 != 0:
      continue

    #roll_start = deg2rad(gps_dat[framenum,7]);
    #pitch_start = -deg2rad(gps_dat[framenum,8]);
    #yaw_start = deg2rad(gps_dat[framenum,9]);

    pitch_start = deg2rad(gps_dat[framenum,7]);
    roll_start = -deg2rad(gps_dat[framenum,8]);
    yaw_start = deg2rad(gps_dat[framenum,9] + 90);

    R_to_i_from_w = \
    euler_matrix(pitch_start,roll_start,yaw_start,'rxyz')[0:3,0:3]

    # pts = zeros([num_imgs_fwd, 3])
    # for t in range(num_imgs_fwd):
    pts = zeros([min(gps_dat.shape[0]-framenum,1000),3])
    for t in range(pts.shape[0]):
      pts[t,:] = WGS84toENU(gps_dat[framenum, 1:4], gps_dat[framenum+t, 1:4])
      pts[t,2] = pts[t,2] 

    for idx in range(1,pts.shape[0]):
      world_coordinates = pts[idx,:]
      pos_wrt_imu = dot(R_to_i_from_w, world_coordinates)
      pos_wrt_camera = dot(R_to_c_from_i, (pos_wrt_imu + array([0.25, -1.1,
        -0.2])))
      #pos_wrt_camera = dot(R_to_c_from_i, (pos_wrt_imu + array([-0.25, -2.13,
      #  0])))
      # quick hack added by Tao
      #pos_wrt_camera[2] = pos_wrt_camera[2]*1.4
      pix = around(dot(cam['KK'], divide(pos_wrt_camera, pos_wrt_camera[2])))
      if (pix[0] > 0 and pix[0] < 1280 and pix[1] > 0 and pix[1] < 960):
        I[pix[1]-3:pix[1]+3, pix[0]-3:pix[0]+3, 0] = 255;
        I[pix[1]-3:pix[1]+3, pix[0]-3:pix[0]+3, 1] = 0;
        I[pix[1]-3:pix[1]+3, pix[0]-3:pix[0]+3, 2] = 0;

    imshow('video', I)
    key = waitKey(5)
    if key == 'q':
      break;


