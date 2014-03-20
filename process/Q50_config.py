import numpy as np
from transformations import euler_matrix

# defines calibration parameters for the Q50 setup. 

# IMU orientation -> x forward, y out the left door, z up
# to convert from WGS84, roll, pitch, azimuth to IMU based cartesian coordinate sysem use: 


# Lidar orientation -> x forward, y out the left door, z up

# camera orientation -> todo

##### LIDAR to IMU calibration parameters #####

# Note: the translation vector is currently unknown
R_from_i_to_l = euler_matrix(0,-0.015,-0.045)[0:3,0:3]
T_from_l_to_i = np.eye(4)
T_from_l_to_i[0:3,0:3] = R_from_i_to_l.transpose()

##### LIDAR to Camera Calibration parameters #####

def GetQ50CameraParams():
    cam = [{}, {}]
    for i in [0, 1]:
        cam[i]['R_to_c_from_i'] = np.array([[-1, 0, 0],
                                         [0, 0, -1],
                                         [0, -1, 0]])

        if i == 0:
            R_to_c_from_l_in_camera_frame = euler_matrix(0.042, 0.0065, -0.002)[0:3,0:3] 
            cam[i]['R_to_c_from_l_in_camera_frame'] = R_to_c_from_l_in_camera_frame
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = np.array([0.0, -0.33, 0.265]);

        elif i == 1:
            R_to_c_from_l_in_camera_frame = euler_matrix(0.0375, 0.00859, 0.0165)[0:3,0:3] 
            cam[i]['R_to_c_from_l_in_camera_frame'] = R_to_c_from_l_in_camera_frame
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = np.array([-0.07, 0.325, 0.16]);


        cam[i]['fx'] = 2221.8
        cam[i]['fy'] = 2233.7
        cam[i]['cu'] = 623.7
        cam[i]['cv'] = 445.7
        cam[i]['KK'] = np.array([[cam[i]['fx'], 0.0, cam[i]['cu']],
                              [0.0, cam[i]['fy'], cam[i]['cv']],
                              [0.0, 0.0, 1.0]])
        cam[i]['f'] = (cam[i]['fx'] + cam[i]['fy']) / 2

    return cam

