import numpy as np
def LoadParameters():
    return GetParams()



def GetLidarParams():
    params = { }
    params['R_from_i_to_l'] = np.reshape(np.array([9.999976e-01, 7.553071e-04, -2.035826e-03, -7.854027e-04, 9.998898e-01, -1.482298e-02, 2.024406e-03, 1.482454e-02, 9.998881e-01]), [3,3])
    params['t_from_i_to_l'] = np.array([-8.086759e-01, 3.195559e-01, -7.997231e-01])
    params['T_from_i_to_l'] = np.eye(4)
    params['T_from_i_to_l'][0:3,0:3] = params['R_from_i_to_l']
    params['T_from_i_to_l'][0:3,3] = params['t_from_i_to_l']
    params['T_from_l_to_i'] = np.linalg.inv(params['T_from_i_to_l'])
    params['height'] = 1.73
    return params


def GetParams():
    folder = ''
    params = { }
    lidarfile = folder+'/calib_imu_to_velo.txt'
    camfile = folder+'/calib_velo_to_cam.txt'
    params['lidar'] = GetLidarParams();
    #params['cam'] = GetCameraParams();
    return params
