import numpy as np
from transformations import euler_matrix

# defines calibration parameters for the Q50 setup.

# IMU orientation -> x forward, y out the left door, z up
# to convert from WGS84, roll, pitch, azimuth to IMU based cartesian coordinate sysem use:


# Lidar orientation -> x forward, y out the left door, z up

# camera orientation -> todo

##### LIDAR to IMU calibration parameters #####

# Note: the translation vector is currently unknown
def GetQ50LidarParams():
    params = { }
    params['R_from_i_to_l'] = euler_matrix(-0.005,-0.0081,-0.0375)[0:3,0:3]
    params['T_from_l_to_i'] = np.eye(4)
    params['T_from_l_to_i'][0:3,0:3] = params['R_from_i_to_l'].transpose()
    params['height']=1.85
    return params

##### LIDAR to Camera Calibration parameters #####

def GetQ50CameraParams():
    cam = [{}, {}]
    for i in [1, 0]:
        cam[i]['R_to_c_from_i'] = np.array([[-1, 0, 0],
                                         [0, 0, -1],
                                         [0, -1, 0]])

        if i == 0:
            cam[i]['R_to_c_from_l_in_camera_frame'] = cam[1]['R_to_c_from_l_in_camera_frame']
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = cam[1]['displacement_from_l_to_c_in_lidar_frame']

            # extrinsics parameters for transforming points in right camera frame to this camera
            T = np.array([-0.5873763710461054, 0.00012196510170337307, 0.08922401781210791])
            R = np.array([0.9999355343485463, -0.00932576944123699, 0.006477435558612815, 0.009223923954826548, 0.9998360945238545, 0.015578938158992275, -0.006621659456863392, -0.01551818647957998, 0.9998576596268203])
            R = R.reshape((3,3))
            T = T.reshape((3,1))
            E = np.hstack((R.transpose(), np.dot(-R.transpose(),T)))
            E = np.vstack((E,np.array([0,0,0,1])))
            cam[i]['E'] = E
            cam[i]['fx'] = 2254.76
            cam[i]['fy'] = 2266.30
            cam[i]['cu'] = 655.55
            cam[i]['cv'] = 488.85
            cam[i]['distort'] = np.array([-0.22146000368016028, 0.7987879799679538, -6.542034918087567e-05, 2.8680581938024014e-05, 0.0])

        elif i == 1:
            R_to_c_from_l_in_camera_frame = euler_matrix(0.046,0.0071,0.0065)[0:3,0:3]
            cam[i]['R_to_c_from_l_in_camera_frame'] = R_to_c_from_l_in_camera_frame
            cam[i]['displacement_from_l_to_c_in_lidar_frame'] = np.array([-0.07, 0.325, 0.16]);
            cam[i]['fx'] = 2250.72
            cam[i]['fy'] = 2263.75
            cam[i]['cu'] = 648.95
            cam[i]['cv'] = 450.24
            cam[i]['distort'] = np.array([-0.16879238412882028, 0.11971166628565273, -0.0017457365846050555, 0.0001853749033525837, 0.0])
            cam[i]['E'] = np.eye(4)



        cam[i]['KK'] = np.array([[cam[i]['fx'], 0.0, cam[i]['cu']],
                              [0.0, cam[i]['fy'], cam[i]['cv']],
                              [0.0, 0.0, 1.0]])
        cam[i]['f'] = (cam[i]['fx'] + cam[i]['fy']) / 2

    return cam

def GetQ50Params():
    params = { }
    params['lidar'] = GetQ50LidarParams();
    params['cam'] = GetQ50CameraParams();
    return params
