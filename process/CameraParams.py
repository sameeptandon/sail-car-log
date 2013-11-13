import numpy as np
import pickle
import sys
from WGS84toENU import *

def getCameraParams():
    cam = [{}, {}]
    for i in [0, 1]:
        cam[i]['R_to_c_from_i'] = np.array([[-1, 0, 0],
                                         [0, 0, -1],
                                         [0, -1, 0]])

        if i == 0:
            cam[i]['rot_x'] = deg2rad(-0.7)  # better cam 1
            cam[i]['rot_y'] = deg2rad(-0.5)
            cam[i]['rot_z'] = deg2rad(-0.005)
            cam[i]['t_x'] = -0.5
            cam[i]['t_y'] = 1.1
            cam[i]['t_z'] = 0.0
        elif i == 1:
            cam[i]['rot_x'] = deg2rad(-0.61)  # better cam 2
            cam[i]['rot_y'] = deg2rad(0.2)
            cam[i]['rot_z'] = deg2rad(0.0)
            cam[i]['t_x'] = 0.5
            cam[i]['t_y'] = 1.1
            cam[i]['t_z'] = 0.0

        cam[i]['fx'] = 2221.8
        cam[i]['fy'] = 2233.7
        cam[i]['cu'] = 623.7
        cam[i]['cv'] = 445.7
        cam[i]['KK'] = np.array([[cam[i]['fx'], 0.0, cam[i]['cu']],
                              [0.0, cam[i]['fy'], cam[i]['cv']],
                              [0.0, 0.0, 1.0]])
        cam[i]['f'] = (cam[i]['fx'] + cam[i]['fy']) / 2

    return cam

if __name__ == '__main__':
    cam= getCameraParams()
    filename = 'cam_params.pickle'
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    pickle.dump(cam, open(filename, 'wb'))

