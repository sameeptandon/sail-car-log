from transformations import euler_matrix
import numpy as np
from WGS84toENU import *
from numpy import array, dot, zeros, around, divide, ones

def R_to_i_from_w(roll, pitch, yaw): 
    cp = cos(pitch)
    sp = sin(pitch)
    ct = cos(roll)
    st = sin(roll)
    cg = cos(yaw)
    sg = sin(yaw)

    R_to_i_from_w = \
            array([[cg*cp-sg*st*sp, -sg*ct, cg*sp+sg*st*cp],
                  [sg*cp+cg*st*sp, cg*ct, sg*sp-cg*st*cp],
                  [-ct*sp, st, ct*cp]]).transpose()

    return R_to_i_from_w; 

def R_to_c_from_i(cam): 
    R_to_c_from_i = cam['R_to_c_from_i']
    R_camera_pitch = euler_matrix(cam['rot_x'], cam['rot_y'], cam['rot_z'], 'sxyz')[0:3,0:3]
    R_to_c_from_i = dot(R_camera_pitch, R_to_c_from_i)

    return R_to_c_from_i

def R_to_c_from_w(roll, pitch, yaw, cam):
    return dot(R_to_c_from_i(cam), R_to_i_from_w(roll, pitch, yaw) )


def integrateVelocity(vel, dt=1/50.0):
    return np.cumsum(vel*dt, axis=0)

def smoothCoordinates(GPSData):
    tr = np.array([np.eye(4),]*GPSData.shape[0])

    roll_start = deg2rad(GPSData[0,8])
    pitch_start = deg2rad(GPSData[0,7])
    yaw_start = -deg2rad(GPSData[0,9])
    base_R_to_i_from_w = R_to_i_from_w(roll_start, pitch_start, yaw_start)

    NEU_coords = integrateVelocity(GPSData[:,4:7])
    ENU_coords = np.array(NEU_coords)
    ENU_coords[:,0] = NEU_coords[:,1]
    ENU_coords[:,1] = NEU_coords[:,0]
    ENU_coords[:,2] = NEU_coords[:,2]

    pos_wrt_imu = np.dot(base_R_to_i_from_w, ENU_coords.transpose())

    tr[:,0,3] = pos_wrt_imu[0,:]
    tr[:,1,3] = pos_wrt_imu[1,:]
    tr[:,2,3] = pos_wrt_imu[2,:]

    for i in xrange(GPSData.shape[0]):
        roll = deg2rad(GPSData[i,8])
        pitch = deg2rad(GPSData[i,7])
        yaw = -deg2rad(GPSData[i,9])

        rot = R_to_i_from_w(roll, pitch, yaw).transpose()
        tr[i, 0:3,0:3] = dot(base_R_to_i_from_w, rot)

    return tr


"""
returns the relative transformations of the IMU from time 0 to the end of the GPS log. 
""" 
def IMUTransforms(GPSData):
    return smoothCoordinates(GPSData)

def GPSTransforms(GPSData, Camera, width=2): 

    tr = np.array([np.eye(4),]*GPSData.shape[0])

    roll_start = -deg2rad(GPSData[0,7]);
    pitch_start = deg2rad(GPSData[0,8]);
    yaw_start = -deg2rad(GPSData[0,9]+90);

    base_R_to_c_from_w = R_to_c_from_w(roll_start, pitch_start, yaw_start, Camera)
    pts = WGS84toENU(GPSData[0,1:4], GPSData[:,1:4])

    world_coordinates = pts;
    pos_wrt_camera = dot(base_R_to_c_from_w, world_coordinates);

    tr[:,0,3] = pos_wrt_camera[0,:]
    tr[:,1,3] = pos_wrt_camera[1,:]
    tr[:,2,3] = pos_wrt_camera[2,:]

    for i in xrange(GPSData.shape[0]):
        roll = -deg2rad(GPSData[i,7])
        pitch = deg2rad(GPSData[i,8])
        yaw = -deg2rad(GPSData[i,9] + 90)

        rot = R_to_c_from_w(roll, pitch, yaw, Camera).transpose()

        pos = pos_wrt_camera[0:3,i]
        tr[i, 0:3,0:3] = dot(base_R_to_c_from_w, rot)

    return tr


