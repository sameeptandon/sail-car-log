import numpy as np
import pickle
import sys, time
from cv2 import imshow, waitKey, imwrite
from scipy.io import loadmat, savemat
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from generate_lane_labels import *
from CameraReprojection import * 
from VideoReader import *
from CameraParams import *
import os
from scipy import spatial
from ParticleFilterDriving import *

## usage: python disp_2d.py <name of video file> <mat file of left/right lane labels> <gps.out of map> <mat file of left/right lane labels of map>

## example: python disp_2d.py 101S_a1.avi 101S_a1_labels.mat 101S_j_gps.out 101S_j1_labels.mat
## note there is no "split_xx" before the video filename, although this file requires split_0 to be present

if __name__ == '__main__':
    #test drive data import
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
    #gps_dat[:,3] = 0
    lastTime = time.time()

    labels = loadmat(sys.argv[2])
    lp = labels['left']
    rp = labels['right']

    dT = np.median(np.diff(gps_dat[:,0]))

    detectorPixels = pickle.load(open(sys.argv[3],'rb'))
 
    map = pickle.load(open(sys.argv[4],'rb'))
    mapENU = WGS84toENU(map[0,:],map).transpose()
    gpsENU = WGS84toENU(map[0,:],gps_dat[:,1:4]).transpose()

    writer = None
    saveImageDest = None
    if len(sys.argv) > 5:
			#saveImageDest = sys.argv[5]
      writer = cv2.VideoWriter(sys.argv[5], cv.CV_FOURCC('F','M','P','4'), 10.0, (640,480))

    cam = getCameraParams()[cam_num - 1] 
    ## Get GPS transforms of both data sets relative to one dataset
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

    """
        lp and rp contain a single pixel location of the lane label in the frame. 
        First, we compute the position of the pixel in 3d w.r.t. the camera using the assumption that the height of the road is known. 

        Then, we do a series of transformations to get the position w.r.t to the position of the camera at time 0. 

        pixel -> pos w.r.t camera in 3d using road height assumption -> pos w.r.t camera at time 0

        The frame is attached to the camera is a standard cartesian XYZ with units of meters and a rotation matrix. It's not ENU or some other convention. 
    """
    left_XYZ = pixelTo3d(lp, cam) 
    right_XYZ = pixelTo3d(rp, cam)
    gps_Pos = np.zeros((lp.shape[0],4))
    left_Pos = np.zeros((lp.shape[0], 4))
    right_Pos = np.zeros((rp.shape[0], 4))
    for t in range(min(lp.shape[0], tr.shape[0])):
      Pos = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([left_XYZ[t,0], left_XYZ[t,1], left_XYZ[t,2], 1])))
      left_Pos[t,:] = Pos
      Pos = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([right_XYZ[t,0], right_XYZ[t,1], right_XYZ[t,2], 1])))
      right_Pos[t,:] = Pos
      Pos = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([0, 0, 0, 1])))
      gps_Pos[t,:] = Pos
    
    '''-----------------------------------'''
    savemat('lanePos',dict(left = left_Pos, right = right_Pos))
    savemat('gpsPos',dict(gpsPos = gps_Pos))
    savemat('gpsDat',dict(gpsData = gps_dat))
    yaw = gps_dat[:,9]
    yaw = -np.pi/180*yaw+np.pi
    #yaw = yaw+np.pi/2-yaw[0]
    yawRate = np.diff(yaw)
    for i in np.where(np.abs(yawRate)>1.9*np.pi)[0]:
      yaw[(i+1):] = yaw[(i+1):]-2*np.pi*np.sign(yawRate[i])
    yawRate = np.diff(yaw)
    #pf = ParticleFilterDriving((gps_Pos[0,0],gps_Pos[0,2],yaw[0]),(5,5,5*pi/180),200,left_Pos,tr,cam_num)
    pf = ParticleFilterDriving((gpsENU[0,0],gpsENU[0,1],yaw[0]),(5,5,5*pi/180),200,mapENU,tr,cam_num)
    #print pf.particles
    #print pf.map
    pfOutput = np.zeros(gps_Pos.shape)


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
        #print lPos2
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
        
        pf.propogate((gps_dat[count,[4,5]]*dT*10,yaw[count],yawRate[count]*10))
        #pf.gpsMeasurement((gps_Pos[count,[0,2]],yaw[count]))
        pf.gpsMeasurement((gpsENU[count,0:2],yaw[count]))
        visionInput = detectorPixels[:,0,:,count].transpose()
        #pf.visionMeasurement(visionInput)
        pfOutput[count,0:3] = pf.state()
        #print pf.state()
        #print (gps_Pos[count,[0,2]],yaw[count])
        #print (gpsENU[count,0:2],yaw[count])

        lpix = detectorPixels[:,0,:,count].transpose()
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
              I[lpix[1,:]+p, lpix[0,:], :] = [0, 255, 0]
              I[lpix[1,:], lpix[0,:]+p, :] = [0, 255, 0]
              I[lpix[1,:]-p, lpix[0,:], :] = [0, 255, 0]
              I[lpix[1,:], lpix[0,:]-p, :] = [0, 255, 0]

        
        lpix = pf.getMapPts(pf.state(),np.zeros(1))
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
              I[lpix[1,:]+p, lpix[0,:], :] = [255, 255, 0]
              I[lpix[1,:], lpix[0,:]+p, :] = [255, 255, 0]
              I[lpix[1,:]-p, lpix[0,:], :] = [255, 255, 0]
              I[lpix[1,:], lpix[0,:]-p, :] = [255, 255, 0]
         

        count += 10
        I = cv2.resize(I, (640, 480))
        if writer:
          writer.write(I) 
        if saveImageDest is not None and count==1000:
          imwrite(saveImageDest,I)
        imshow('video', I)
        key = waitKey(10)
        if key == ord('q'):
            break
        if time.time() - lastTime > 1:
          print 'framenum = ', count
          lastTime = time.time()

        if count==10000:
          savemat('pfOutput',dict(pfOut = pfOutput))
