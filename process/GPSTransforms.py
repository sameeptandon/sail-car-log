

from transformations import euler_matrix
import numpy as np
from WGS84toENU import *
from numpy import array, dot, zeros, around, divide, ones


""" 
R_to_i_from_w computes the rotation matrix from the SPAN-SE IMU log. This
matrix transforms ENU coordinates into XYZ w.r.t the IMU (the axis of the IMU
are on the unit)
"""
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

""" 
Numerical integration of velocity given a fixed dt
""" 
def integrateVelocity(vel, dt):
    return np.cumsum(vel*dt, axis=0)

"""
Smooth coordinates integrates the velocity off the IMU to get position
estimates. It uses absolute orientation estimates off the IMU. 

The function returns Tx4x4 transforms w.r.t. to the position encoded by
GPSData[0, :]
"""
def smoothCoordinates(GPSData, dt):
    tr = np.array([np.eye(4),]*GPSData.shape[0])

    roll_start = deg2rad(GPSData[0,8])
    pitch_start = deg2rad(GPSData[0,7])
    yaw_start = -deg2rad(GPSData[0,9])
    base_R_to_i_from_w = R_to_i_from_w(roll_start, pitch_start, yaw_start)

    NEU_coords = integrateVelocity(GPSData[:,4:7], dt=dt)
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
Returns a Tx4x4 transform of the IMU position w.r.t IMU position at time 0. 
It's generally better to use smoothCoordinates for everything other than
global mapping. This function is useful for computing the difference between
two or more locations that are not linked via velocity (i.e. two starting
positions for different gps logs)
"""
def absoluteTransforms(GPSData): 
    tr = np.array([np.eye(4),]*GPSData.shape[0])

    roll_start = deg2rad(GPSData[0,8])
    pitch_start = deg2rad(GPSData[0,7])
    yaw_start = -deg2rad(GPSData[0,9])
    base_R_to_i_from_w = R_to_i_from_w(roll_start, pitch_start, yaw_start)

    ENU_coords = WGS84toENU(GPSData[0,1:4], GPSData[:,1:4])
    pos_wrt_imu = np.dot(base_R_to_i_from_w, ENU_coords)

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
returns the relative transformations of the IMU from time 0 to the end of the
GPS log. 
""" 
def IMUTransforms(GPSData):
    dt = GPSData[1, 0] - GPSData[0, 0]
    return smoothCoordinates(GPSData, dt=dt)

