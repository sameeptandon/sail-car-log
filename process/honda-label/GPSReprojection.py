from transformations import euler_matrix
import numpy as np
from WGS84toENU import *
from numpy import array, dot, zeros, around, divide, ones

def GPSVelocities(GPSData):
   return (np.apply_along_axis(np.linalg.norm, 1, GPSData[:,4:7]))

def GPSPos(GPSData, Camera, start_frame):
    roll_start = -deg2rad(start_frame[7]);
    pitch_start = deg2rad(start_frame[8]);
    yaw_start = -deg2rad(start_frame[9]+90);

    psi = pitch_start; 
    cp = cos(psi);
    sp = sin(psi);
    theta = roll_start;
    ct = cos(theta);
    st = sin(theta);
    gamma = yaw_start;
    cg = cos(gamma);
    sg = sin(gamma);

    R_to_i_from_w = \
            array([[cg*cp-sg*st*sp, -sg*ct, cg*sp+sg*st*cp],
                  [sg*cp+cg*st*sp, cg*ct, sg*sp-cg*st*cp],
                  [-ct*sp, st, ct*cp]]).transpose()


    pts = WGS84toENU(start_frame[1:4], GPSData[:, 1:4])
    world_coordinates = pts;
    pos_wrt_imu = dot(R_to_i_from_w, world_coordinates);
    R_to_c_from_i = Camera['R_to_c_from_i']
    R_camera_pitch = euler_matrix(Camera['rot_x'], Camera['rot_y'],\
            Camera['rot_z'], 'sxyz')[0:3,0:3]
    R_to_c_from_i = dot(R_camera_pitch, R_to_c_from_i) 

    pos_wrt_camera = dot(R_to_c_from_i, pos_wrt_imu);

    pos_wrt_camera[0,:] += Camera['t_x'] #move to left/right
    pos_wrt_camera[1,:] += Camera['t_y'] #move up/down image
    pos_wrt_camera[2,:] += Camera['t_z'] #move away from cam
    return pos_wrt_camera



def GPSPosShifted(GPSData, Camera, start_frame):
    roll_start = -deg2rad(start_frame[7]);
    pitch_start = deg2rad(start_frame[8]);
    yaw_start = -deg2rad(start_frame[9]+90);

    psi = pitch_start; 
    cp = cos(psi);
    sp = sin(psi);
    theta = roll_start;
    ct = cos(theta);
    st = sin(theta);
    gamma = yaw_start;
    cg = cos(gamma);
    sg = sin(gamma);

    R_to_i_from_w = \
            array([[cg*cp-sg*st*sp, -sg*ct, cg*sp+sg*st*cp],
                  [sg*cp+cg*st*sp, cg*ct, sg*sp-cg*st*cp],
                  [-ct*sp, st, ct*cp]]).transpose()


    pts = WGS84toENU(start_frame[1:4], GPSData[:, 1:4])
    vel = GPSData[:,4:7]
    vel[:,[0, 1]] = vel[:,[1, 0]]
    sideways = np.cross(vel, np.array([0,0,1]), axisa=1)
    sideways/= np.sqrt((sideways ** 2).sum(-1))[..., np.newaxis]
    #pts[0,:] = pts[0,:]+np.transpose(GPSData[:,4])/40
    #pts[1,:] = pts[1,:]-np.transpose(GPSData[:,5])/40
    pts = pts+sideways.transpose()
    world_coordinates = pts;
    pos_wrt_imu = dot(R_to_i_from_w, world_coordinates);
    R_to_c_from_i = Camera['R_to_c_from_i']
    R_camera_pitch = euler_matrix(Camera['rot_x'], Camera['rot_y'],\
            Camera['rot_z'], 'sxyz')[0:3,0:3]
    R_to_c_from_i = dot(R_camera_pitch, R_to_c_from_i) 

    pos_wrt_camera = dot(R_to_c_from_i, pos_wrt_imu);

    pos_wrt_camera[0,:] += Camera['t_x'] #move to left/right
    pos_wrt_camera[1,:] += Camera['t_y'] #move up/down image
    pos_wrt_camera[2,:] += Camera['t_z'] #move away from cam
    return pos_wrt_camera



def ENU2IMU(world_coordinates, start_frame):
    roll_start = -deg2rad(start_frame[7]);
    pitch_start = deg2rad(start_frame[8]);
    yaw_start = -deg2rad(start_frame[9]+90);

    psi = pitch_start; 
    cp = cos(psi);
    sp = sin(psi);
    theta = roll_start;
    ct = cos(theta);
    st = sin(theta);
    gamma = yaw_start;
    cg = cos(gamma);
    sg = sin(gamma);

    R_to_i_from_w = \
            array([[cg*cp-sg*st*sp, -sg*ct, cg*sp+sg*st*cp],
                  [sg*cp+cg*st*sp, cg*ct, sg*sp-cg*st*cp],
                  [-ct*sp, st, ct*cp]]).transpose()
    pos_wrt_imu = dot(R_to_i_from_w, world_coordinates);
    return pos_wrt_imu


def GPSPosIMU(GPSData, Camera, start_frame):
    roll_start = -deg2rad(start_frame[7]);
    pitch_start = deg2rad(start_frame[8]);
    yaw_start = -deg2rad(start_frame[9]+90);

    psi = pitch_start; 
    cp = cos(psi);
    sp = sin(psi);
    theta = roll_start;
    ct = cos(theta);
    st = sin(theta);
    gamma = yaw_start;
    cg = cos(gamma);
    sg = sin(gamma);

    R_to_i_from_w = \
            array([[cg*cp-sg*st*sp, -sg*ct, cg*sp+sg*st*cp],
                  [sg*cp+cg*st*sp, cg*ct, sg*sp-cg*st*cp],
                  [-ct*sp, st, ct*cp]]).transpose()
    pts = WGS84toENU(start_frame[1:4], GPSData[:, 1:4])
    world_coordinates = pts;
    pos_wrt_imu = dot(R_to_i_from_w, world_coordinates);
    return pos_wrt_imu

def GPSPosCamera(pos_wrt_imu, Camera):
    R_to_c_from_i = Camera['R_to_c_from_i']
    R_camera_pitch = euler_matrix(Camera['rot_x'], Camera['rot_y'],\
            Camera['rot_z'], 'sxyz')[0:3,0:3]
    R_to_c_from_i = dot(R_camera_pitch, R_to_c_from_i) 

    pos_wrt_camera = dot(R_to_c_from_i, pos_wrt_imu);

    pos_wrt_camera[0,:] += Camera['t_x'] #move to left/right
    pos_wrt_camera[1,:] += Camera['t_y'] #move up/down image
    pos_wrt_camera[2,:] += Camera['t_z'] #move away from cam
    return pos_wrt_camera




def GPSColumns(GPSData, Camera, start_frame):
    pos_wrt_camera = GPSPos(GPSData, Camera, start_frame)
    return PointsMask(pos_wrt_camera[:,1:], Camera)

def GPSShiftedColumns(GPSData, Camera, start_frame):
    pos_wrt_camera = GPSPosShifted(GPSData, Camera, start_frame)
    return PointsMask(pos_wrt_camera, Camera)

def PointsMask(pos_wrt_camera, Camera):
    vpix = around(dot(Camera['KK'], divide(pos_wrt_camera, pos_wrt_camera[2,:])))
    vpix = vpix.astype(np.int32)
    return vpix
   

def GPSMask(GPSData, Camera, width=2): 
    I = 255*ones((960,1280,3), np.uint8)
    vpix = GPSColumns(GPSData, Camera, GPSData[0, :])
    vpix = vpix[:,vpix[0,:] > 0 + width/2]
    if vpix.size > 0:
      vpix = vpix[:,vpix[1,:] > 0 + width/2]
    if vpix.size > 0:
      vpix = vpix[:,vpix[0,:] < 1279 - width/2]
    if vpix.size > 0:
      vpix = vpix[:,vpix[1,:] < 959 - width/2]
    
      for p in range(-width/2,width/2):
          I[vpix[1,:]+p, vpix[0,:], :] = 0
          I[vpix[1,:], vpix[0,:]+p, :] = 0
          I[vpix[1,:]-p, vpix[0,:], :] = 0
          I[vpix[1,:], vpix[0,:]-p, :] = 0

    """
    for idx in range(1,pts.shape[1]):
      pix = vpix[:,idx]
      if (pix[0] > 0 and pix[0] < 1280 and pix[1] > 0 and pix[1] < 960):
        I[pix[1]-width+1:pix[1]+width, pix[0]-width+1:pix[0]+width, 0] = 0;
        I[pix[1]-width+1:pix[1]+width, pix[0]-width+1:pix[0]+width, 1] = 0;
        I[pix[1]-width+1:pix[1]+width, pix[0]-width+1:pix[0]+width, 2] = 0;
    """
    
    return I

