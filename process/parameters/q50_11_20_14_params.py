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
    params['height'] = 2.0
    return params

##### LIDAR to Camera Calibration parameters #####

def GetQ50CameraParams():
    cam = {}
    for i in [601, 604]:
        cam[i] = {}
        cam[i]['R_to_c_from_i'] = np.array([[-1, 0, 0],
                                            [0, 0, -1],
                                            [0, -1, 0]])

        if i == 601: # right camera
            R_to_c_from_l_in_camera_frame = euler_matrix(0.022500,-0.006000,-0.009500)[0:3,0:3]
            # R_to_c_from_l_in_camera_frame = euler_matrix(0.037000,0.025500,-0.001000)[0:3,0:3]
            cam[i]['R_to_c_from_l_in_camera_frame'] = R_to_c_from_l_in_camera_frame
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = \
                np.array([-0.522, 0.2925, 0.273])
            cam[i]['E'] = np.eye(4)
            cam[i]['width'] = 1280
            cam[i]['height'] = 800
            cam[i]['fx'] = 1.95205250e+03
            cam[i]['fy'] = 1.95063141e+03
            cam[i]['cu'] = 6.54927553e+02
            cam[i]['cv'] = 4.01883041e+02
            cam[i]['distort'] = np.array([-3.90035052e-01, 6.85186191e-01, 3.22989088e-03, 1.02017567e-03])
            # cam[i]['fx'] = 2.03350359e+03
            # cam[i]['fy'] = 2.03412303e+03
            # cam[i]['cu'] = 5.90322443e+02
            # cam[i]['cv'] = 4.30538351e+02
            # cam[i]['distort'] = np.array([-4.36806404e-01, 2.16674616e+00, -1.02905742e-02, 1.81030435e-03, -1.05642273e+01])

        elif i == 604: # left camera
            R_to_c_from_l_in_camera_frame = euler_matrix(0.025000,-0.011500,0.000000)[0:3,0:3]
            cam[i]['R_to_c_from_l_in_camera_frame'] = R_to_c_from_l_in_camera_frame
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = \
                np.array([-0.522, -0.2925, 0.273])
            cam[i]['E'] = np.eye(4)
            cam[i]['width'] = 1280
            cam[i]['height'] = 800
            cam[i]['fx'] = 1.95205250e+03
            cam[i]['fy'] = 1.95063141e+03
            cam[i]['cu'] = 6.54927553e+02
            cam[i]['cv'] = 4.01883041e+02
            cam[i]['distort'] = np.array([-3.90035052e-01, 6.85186191e-01, 3.22989088e-03, 1.02017567e-03])

        cam[i]['KK'] = np.array([[cam[i]['fx'], 0.0, cam[i]['cu']],
                                 [0.0, cam[i]['fy'], cam[i]['cv']],
                                 [0.0, 0.0, 1.0]])
        cam[i]['f'] = (cam[i]['fx'] + cam[i]['fy']) / 2

    return cam


def GetQ50RadarParams():
    params = { }
    params['R_from_r_to_l'] = euler_matrix(0, 0, -.009)[0:3,0:3]
    params['T_from_r_to_l'] = [3.17, 0.4, -1.64]

    return params

def GetQ50Params():
    params = { }
    params['lidar'] = GetQ50LidarParams();
    params['cam'] = GetQ50CameraParams();
    params['radar'] = GetQ50RadarParams();
    return params
