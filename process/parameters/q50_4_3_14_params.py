import numpy as np
from transformations import euler_matrix

# defines calibration parameters for the Q50 setup. 

# IMU orientation -> x forward, y out the left door, z up
# to convert from WGS84, roll, pitch, azimuth to IMU based cartesian coordinate sysem use: 


# Lidar orientation -> x forward, y out the left door, z up

# camera orientation -> todo

##### LIDAR to IMU calibration parameters #####

def GetQ50LidarParams():
    params = { }
    params['R_from_i_to_l'] = euler_matrix(-0.04, -0.0146, -0.0165)[0:3,0:3]
    params['T_from_l_to_i'] = np.eye(4)
    params['T_from_l_to_i'][0:3,0:3] = params['R_from_i_to_l'].transpose()

    return params

##### LIDAR to Camera Calibration parameters #####

def GetQ50CameraParams():
    cam = [{}, {}]
    for i in [0, 1]:
        cam[i]['R_to_c_from_i'] = np.array([[-1, 0, 0],
                                         [0, 0, -1],
                                         [0, -1, 0]])

        if i == 0: # left camera
            
            R_to_c_from_l_in_camera_frame = euler_matrix(0.044,0.0291,0.0115)[0:3,0:3] 
            cam[i]['R_to_c_from_l_in_camera_frame'] = R_to_c_from_l_in_camera_frame
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = np.array([-0.5,0.31,0.34]);
            cam[i]['fx'] = 2254.76 # these parameters for this camera are not updated
            cam[i]['fy'] = 2266.30
            cam[i]['cu'] = 655.55
            cam[i]['cv'] = 488.85
            cam[i]['distort'] = np.array([-0.22146000368016028, 0.7987879799679538, -6.542034918087567e-05, 2.8680581938024014e-05, 0.0])

        elif i == 1: # right camera
            R_to_c_from_l_in_camera_frame = euler_matrix(0.044,0.0291,0.0115)[0:3,0:3] 
            cam[i]['R_to_c_from_l_in_camera_frame'] = R_to_c_from_l_in_camera_frame
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = np.array([-0.5,0.31,0.34]);

            cam[i]['fx'] = 2250.72
            cam[i]['fy'] = 2263.75
            cam[i]['cu'] = 648.95
            cam[i]['cv'] = 450.24
            cam[i]['distort'] = np.array([-0.16879238412882028, 0.11971166628565273, -0.0017457365846050555, 0.0001853749033525837, 0.0])

        cam[i]['KK'] = np.array([[cam[i]['fx'], 0.0, cam[i]['cu']],
                              [0.0, cam[i]['fy'], cam[i]['cv']],
                              [0.0, 0.0, 1.0]])
        cam[i]['f'] = (cam[i]['fx'] + cam[i]['fy']) / 2

    return cam


def GetQ50RadarParams():
    params = { }
    params['R_from_r_to_l'] = euler_matrix(0, 0, -.015)[0:3,0:3]
    params['T_from_r_to_l'] = [3.17, 0.4, -1.64]

    return params

def GetQ50Params(): 
    params = { }
    params['lidar'] = GetQ50LidarParams();
    params['cam'] = GetQ50CameraParams();
    params['radar'] = GetQ50RadarParams();
    return params
