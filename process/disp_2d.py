import numpy as np
import sys, time
from cv2 import imshow, waitKey
from scipy.io import loadmat
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from generate_lane_labels import *
from CameraReprojection import * 
from VideoReader import *

if __name__ == '__main__':
    video_filename = sys.argv[1]
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    num_imgs_fwd = 125
    base_interp_length = 10
    polynomial_fit = 1
    thickness = 2
    width = 10
    video_reader = VideoReader(video_filename,num_splits=1)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()
    lastTime = time.time()

    labels = loadmat(sys.argv[2])
    lp = labels['left']
    rp = labels['right']
  
    writer = None
    if len(sys.argv) > 3:
      writer = cv2.VideoWriter(sys.argv[3], cv.CV_FOURCC('F','M','P','4'), 10.0, (640,480))


    

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

    cam['fx'] = 2221.8
    cam['fy'] = 2233.7
    cam['cu'] = 623.7
    cam['cv'] = 445.7
    cam['KK'] = array([[cam['fx'], 0.0, cam['cu']], \
                     [0.0, cam['fy'], cam['cv']], \
                     [0.0, 0.0, 1.0]]);

    tr = GPSTransforms(gps_dat, cam)
  
    pitch = -cam['rot_x']
    height = 1.106
    # probably have to change these
    f = (cam['fx'] + cam['fy']) / 2
    R_to_c_from_i = cam['R_to_c_from_i']
    R_camera_pitch = euler_matrix(cam['rot_x'], cam['rot_y'],\
            cam['rot_z'], 'sxyz')[0:3,0:3]
    R_to_c_from_i = dot(R_camera_pitch, R_to_c_from_i)
    Tc = np.eye(4)
    Tc[0:3, 0:3] = np.transpose(R_to_c_from_i)
    #Tc[1, 3] -= height
    Tc = np.array([[1, 0, 0, 0], [0, np.cos(pitch), -np.sin(pitch), -height], [0, np.sin(pitch), np.cos(pitch), 0], [0, 0, 0, 1]])

    Tc2 = np.eye(4) # check testTrackReverse for actual transformation value

    left_XYZ = pixelTo3d(lp, cam)
    right_XYZ = pixelTo3d(rp, cam)
    left_Pos = np.zeros((lp.shape[0], 4))
    right_Pos = np.zeros((rp.shape[0], 4))
    for t in range(lp.shape[0]):
      Pos = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([left_XYZ[t,0], left_XYZ[t,1], left_XYZ[t,2], 1])))
      left_Pos[t,:] = Pos
      Pos = np.dot(tr[t, :, :], np.linalg.solve(Tc, np.array([right_XYZ[t,0], right_XYZ[t,1], right_XYZ[t,2], 1])))
      right_Pos[t,:] = Pos

    count = 0
    start = 0
    while True:
        (success, I) = video_reader.getNextFrame()

        if not success:
            break
        """
        if count % 10 != 0: 
            count += 1
            continue
        """

        if count > lp.shape[0] or count > rp.shape[0]:
            break

        start = count
        end = min(lp.shape[0], start+num_imgs_fwd)-1
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

        left_points = np.zeros((960, 1280))
        right_points = np.zeros((960, 1280))
    

        lpts = left_Pos[start:end,:];
        lpts = lpts[lp[start:end,0] > 0, :]
        lPos2 = np.linalg.solve(tr[count, :, :], lpts.transpose())
        lpix = np.around(np.dot(cam['KK'], np.divide(lPos2[0:3,:], lPos2[2, :])))
        if lpix.shape[1] > 0:
          lpix = lpix.astype(np.int32)
          lpix = lpix[:,lpix[0,:] > 0 + width/2]
          lpix = lpix[:,lpix[1,:] > 0 + width/2]
          lpix = lpix[:,lpix[0,:] < 1279 - width/2]
          lpix = lpix[:,lpix[1,:] < 959 - width/2]
    
          for p in range(-width/2,width/2):
            I[lpix[1,:]+p, lpix[0,:], :] = [0, 255, 255]
            I[lpix[1,:], lpix[0,:]+p, :] = [0, 255, 255]
            I[lpix[1,:]-p, lpix[0,:], :] = [0, 255, 255]
            I[lpix[1,:], lpix[0,:]-p, :] = [0, 255, 255]

          lpix_base = lpix[:, :base_interp_length]
          l_p_fit = np.polyfit(lpix_base[1, :], lpix_base[0, :], polynomial_fit)
          l_y_output = np.arange(int(np.max(lpix_base[1, :])), 960 - width / 2)  # int(np.max(left_y) + 1))
          l_x_output = np.polyval(l_p_fit, l_y_output)
          l_x_output = l_x_output.astype(np.int32)
          l_y_output = l_y_output.astype(np.int32)
          l_y_output = l_y_output[l_x_output >= width / 2]
          l_x_output = l_x_output[l_x_output >= width / 2]
          l_y_output = l_y_output[l_x_output < 1280 - width / 2 - 1]
          l_x_output = l_x_output[l_x_output < 1280 - width / 2 - 1]
          for p in range(-width/2,width/2):
            I[l_y_output+p, l_x_output, :] = [255, 0, 0]
            I[l_y_output, l_x_output+p, :] = [255, 0, 0]
            I[l_y_output-p, l_x_output, :] = [255, 0, 0]
            I[l_y_output, l_x_output-p, :] = [255, 0, 0]
      
        rpts = right_Pos[start:end,:];
        rpts = rpts[rp[start:end,0] > 0, :]
        rPos2 = np.linalg.solve(tr[count, :, :], rpts.transpose())
        rpix = np.around(np.dot(cam['KK'], np.divide(rPos2[0:3,:], rPos2[2, :])))

        if rpix.shape[1] > 0:
          rpix = rpix.astype(np.int32)
          rpix = rpix[:,rpix[0,:] > 0 + width/2]
          rpix = rpix[:,rpix[1,:] > 0 + width/2]
          rpix = rpix[:,rpix[0,:] < 1279 - width/2]
          rpix = rpix[:,rpix[1,:] < 959 - width/2]
          rpix_base = rpix[:, :base_interp_length]
    
          for p in range(-width/2,width/2):
            I[rpix[1,:]+p, rpix[0,:], :] = [0, 255, 255]
            I[rpix[1,:], rpix[0,:]+p, :] = [0, 255, 255]
            I[rpix[1,:]-p, rpix[0,:], :] = [0, 255, 255]
            I[rpix[1,:], rpix[0,:]-p, :] = [0, 255, 255]

          rpix_base = rpix[:, :base_interp_length]
          r_p_fit = np.polyfit(rpix_base[1, :], rpix_base[0, :], polynomial_fit)
          r_y_output = np.arange(int(np.max(rpix_base[1, :])), 960 - width / 2)  # int(np.max(reft_y) + 1))
          r_x_output = np.polyval(r_p_fit, r_y_output)
          r_x_output = r_x_output.astype(np.int32)
          r_y_output = r_y_output.astype(np.int32)
          r_y_output = r_y_output[r_x_output > 0 + width / 2] 
          r_x_output = r_x_output[r_x_output > 0 + width / 2]
          r_y_output = r_y_output[r_x_output < 1280 - width / 2 - 1]
          r_x_output = r_x_output[r_x_output < 1280 - width / 2 - 1]
          for p in range(-width/2,width/2):
            I[r_y_output+p, r_x_output, :] = [255, 0, 0]
            I[r_y_output, r_x_output+p, :] = [255, 0, 0]
            I[r_y_output-p, r_x_output, :] = [255, 0, 0]
            I[r_y_output, r_x_output-p, :] = [255, 0, 0]

        count += 10
        I = cv2.resize(I, (640, 480))
        if writer:
          writer.write(I) 
        imshow('video', I)
        key = waitKey(1)
        if key == ord('q'):
            break
        if time.time() - lastTime > 5:
          print 'framenum = ', count
          lastTime = time.time()

