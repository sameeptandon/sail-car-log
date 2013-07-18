import sys
from GPSReader import *
from VideoReader import *
from WGS84toENU import *
from GPSReprojection import *
from transformations import euler_matrix
from numpy import array, dot, zeros, around, divide
from cv2 import imshow, waitKey, resize
import time

if __name__ == '__main__':
  video_filename = sys.argv[1]
  gps_filename = sys.argv[2]
  num_imgs_fwd = 1250; 
  video_reader = VideoReader(video_filename)
  gps_reader = GPSReader(gps_filename)
  gps_dat = gps_reader.getNumericData()

  cam = { }
  cam['R_to_c_from_i'] = array([[-1, 0, 0], \
                         [0, 0, -1], \
                         [0, -1, 0]]);
  #cam['rot_x'] = deg2rad(-0.62); # good cam2
  #cam['rot_y'] = deg2rad(0.5);
  #cam['rot_z'] = deg2rad(0.027);
  cam['rot_x'] = deg2rad(-0.70);
  cam['rot_y'] = deg2rad(-0.5);
  cam['rot_z'] = deg2rad(0.0);
  #cam['rot_x'] = deg2rad(-0.66); # good cam 1
  #cam['rot_y'] = deg2rad(-0.71);
  #cam['rot_z'] = deg2rad(-0.005);


  cam['fx'] = 2221.8
  cam['fy'] = 2233.7
  cam['cu'] = 623.7
  cam['cv'] = 445.7
  cam['KK'] = array([[cam['fx'], 0.0, cam['cu']], \
                     [0.0, cam['fy'], cam['cv']], \
                     [0.0, 0.0, 1.0]]);

  #framenum = 1926;
  framenum = 0
  lastTime = time.time()
  #video_reader.setFrame(framenum)
  while True:
    framenum = framenum + 1;
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    if framenum % 1 != 0:
      continue

    M = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, width=10); 
    I = np.minimum(M,I)
    I = resize(I,(640, 480))
    #imshow('video', I)
    key = waitKey(1)
    if key == ord('q'):
      break;
    currentTime = time.time();
    if (currentTime - lastTime > 1):
        print framenum
        lastTime = currentTime

