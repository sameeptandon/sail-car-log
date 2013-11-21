import numpy as np
import pickle
import sys, time
from cv2 import imshow, waitKey
from scipy.io import loadmat, savemat
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from generate_lane_labels import *
from CameraReprojection import * 
from VideoReader import *
from CameraParams import * 

## usage: python disp_2d.py <name of video file> <mat file of left/right lane labels>

## example: python disp_2d.py 101S_a1.avi 101S_a1_labels.mat 
## note there is no "split_xx" before the video filename, although this file requires split_0 to be present

if __name__ == '__main__':
    video_filename = sys.argv[1] 
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    num_imgs_fwd = 125
    width = 10
    video_reader = VideoReader(video_filename,num_splits=1) # require only split_0 
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()
    lastTime = time.time()

    labels = loadmat(sys.argv[2])
    lp = labels['left']
    rp = labels['right']
  
    writer = None
    if len(sys.argv) > 3:
      writer = cv2.VideoWriter(sys.argv[3], cv.CV_FOURCC('F','M','P','4'), 10.0, (640,480))

    cam = getCameraParams()[cam_num - 1] 
    tr = GPSTransforms(gps_dat, cam)

    #pitch = -cam['rot_x']
    height = 1.106 
    # probably have to change these
    R_camera_pitch = euler_matrix(cam['rot_x'], cam['rot_y'],\
            cam['rot_z'], 'sxyz')[0:3,0:3]
    Tc = np.eye(4)
    #Tc[0:3, 0:3] = np.transpose(R_camera_pitch)
    #Tc[1, 3] -= height
    #Tc[0, 3] -= 0.2
    #Tc[2, 3] -= 0.5
    #Tc = np.array([[1, 0, 0, 0], [0, np.cos(pitch), -np.sin(pitch), -height], [0, np.sin(pitch), np.cos(pitch), 0], [0, 0, 0, 1]])

    """
        lp and rp contain a single pixel location of the lane label in the frame. 
        First, we compute the position of the pixel in 3d w.r.t. the camera using the assumption that the height of the road is known. 

        Then, we do a series of transformations to get the position w.r.t to the position of the camera at time 0. 

        pixel -> pos w.r.t camera in 3d using road height assumption -> pos w.r.t camera at time 0

        The frame is attached to the camera is a standard cartesian XYZ with units of meters and a rotation matrix. It's not ENU or some other convention. 
    """
    left_XYZ = pixelTo3d(lp, cam) 
    right_XYZ = pixelTo3d(rp, cam)
    left_Pos = np.zeros((lp.shape[0], 4))
    right_Pos = np.zeros((rp.shape[0], 4))
    for t in range(min(lp.shape[0], tr.shape[0])):
      Pos = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([left_XYZ[t,0], left_XYZ[t,1], left_XYZ[t,2], 1])))
      left_Pos[t,:] = Pos
      Pos = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([right_XYZ[t,0], right_XYZ[t,1], right_XYZ[t,2], 1])))
      right_Pos[t,:] = Pos

    '''-----------------------------------'''


    count = 0
    start = 0
    while True:
        (success, I) = video_reader.getNextFrame()

        if not success:
            break

        if count > lp.shape[0] or count > rp.shape[0]:
            break

        start = count
        end = min(lp.shape[0], start+num_imgs_fwd)-1

        """ compute the center GPS trajectory reprojection """ 
        framenums_to_reproject = np.arange(start,end)
        special_frame_idx = framenums_to_reproject % 50 == 0 
        for frame_depth in range(1,5):
          special_frame_idx = np.logical_or(framenums_to_reproject % 50 == frame_depth, special_frame_idx)
        special_frames = framenums_to_reproject[special_frame_idx]

        if start not in special_frames:
          special_frames = np.concatenate([[start], special_frames])
    
        M = GPSMask(gps_dat[framenums_to_reproject,:], cam, width=5); 
        I = np.minimum(M,I)
        M = 255 - M;
        I[:,:,2] = np.maximum(M[:,:,2], I[:,:,2])
        M = GPSMask(gps_dat[special_frames,:], cam, width=5); 
        I = np.minimum(M,I)
        M = 255 - M;
        I[:,:,1] = np.maximum(M[:,:,1], I[:,:,1])

        """ code to compute the left lane reprojection"""  
        left_points = np.zeros((960, 1280))
        right_points = np.zeros((960, 1280))

        lpts = left_Pos[start:end,:]; # select points in time range
        lpts = lpts[lp[start:end,0] > 0, :] 
        lPos2 = np.linalg.solve(tr[count, :, :], lpts.transpose())
        lpix = np.around(np.dot(cam['KK'], np.divide(lPos2[0:3,:], lPos2[2, :])))
        if lpix.shape[1] > 0:
          lpix = lpix.astype(np.int32)
          lpix = lpix[:,lpix[0,:] > 0 + width/2]
          if lpix.size > 0:
            lpix = lpix[:,lpix[1,:] > 0 + width/2]
          if lpix.size > 0:
            lpix = lpix[:,lpix[0,:] < 1279 - width/2]
          if lpix.size > 0:
            lpix = lpix[:,lpix[1,:] < 959 - width/2]
    
            for p in range(-width/2,width/2):
              I[lpix[1,:]+p, lpix[0,:], :] = [0, 255, 255]
              I[lpix[1,:], lpix[0,:]+p, :] = [0, 255, 255]
              I[lpix[1,:]-p, lpix[0,:], :] = [0, 255, 255]
              I[lpix[1,:], lpix[0,:]-p, :] = [0, 255, 255]

        
        """ code to compute the right lane reprojection"""  
        rpts = right_Pos[start:end,:];
        rpts = rpts[rp[start:end,0] > 0, :]
        rPos2 = np.linalg.solve(tr[count, :, :], rpts.transpose())
        rpix = np.around(np.dot(cam['KK'], np.divide(rPos2[0:3,:], rPos2[2, :])))

        if rpix.shape[1] > 0:
          rpix = rpix.astype(np.int32)
          rpix = rpix[:,rpix[0,:] > 0 + width/2]
          if rpix.size > 0:
            rpix = rpix[:,rpix[1,:] > 0 + width/2]
          if rpix.size > 0:
            rpix = rpix[:,rpix[0,:] < 1279 - width/2]
          if rpix.size > 0:
            rpix = rpix[:,rpix[1,:] < 959 - width/2]
    
            for p in range(-width/2,width/2):
              I[rpix[1,:]+p, rpix[0,:], :] = [0, 255, 255]
              I[rpix[1,:], rpix[0,:]+p, :] = [0, 255, 255]
              I[rpix[1,:]-p, rpix[0,:], :] = [0, 255, 255]
              I[rpix[1,:], rpix[0,:]-p, :] = [0, 255, 255]

        count += 10
        I = cv2.resize(I, (640, 480))
        if writer:
          writer.write(I) 
        imshow('video', I)
        key = waitKey(10)
        if key == ord('q'):
            break
        if time.time() - lastTime > 1:
          print 'framenum = ', count
          lastTime = time.time()

