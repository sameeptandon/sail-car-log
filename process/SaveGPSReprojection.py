import sys, os
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
  path, vfname = os.path.split(video_filename)
  vidname = vfname.split('.')[0]
  cam_num = int(vidname[-1])
  gps_filename = path + '/' + vidname[0:-1] + '_gps.out' 
  print video_filename
  print gps_filename
  output_dir = sys.argv[2]
  output_filename = output_dir + '/' + vfname
  if len(sys.argv) > 3:
    output_width = int(sys.argv[3])
    output_height = int(sys.argv[4])
  else:
    output_width = 1280
    output_height = 960

  num_imgs_fwd = 1000; 
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

  writer = cv2.VideoWriter(output_filename, cv.CV_FOURCC('F','M','P','4'), 50.0, (output_width, output_height))

  framenum = 0
  frameRate = 0 
  lastTime = time.time()
  #video_reader.setFrame(framenum)
  while True:
    framenum = framenum + 1;
    frameRate = frameRate + 1; 
    (success, I) = video_reader.getNextFrame()
    if success == False:
      break
    if framenum + num_imgs_fwd + 1 > gps_dat.shape[0]:
      break

    M = GPSMask(gps_dat[framenum:framenum+num_imgs_fwd,:], cam, width=1); 
    I = np.minimum(M,I)
    I = resize(I,(output_width, output_height))
    #imshow('video', I)
    #key = waitKey(10)
    writer.write(I);

    #if key == ord('q'):
    #  break;
    currentTime = time.time();
    if (currentTime - lastTime > 10):
        print 'Encoding at', frameRate / 10,  'Hz'
        lastTime = currentTime
        frameRate = 0

  print 'Done encoding: ', video_filename 
