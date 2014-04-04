import numpy as np
import os


def loadLDR(ldrfile):
    z = np.fromfile(ldrfile, dtype=np.float32)
    z = z.reshape((z.shape[0] / 5, 5))
    return z

def loadPCD(pcdfile): 
    f = open(pcdfile, 'r')
    count = 0
    data_started = False
    for l in f:
        t = l.split()
        if t[0] == 'POINTS':
            num_pts = int(t[1])
            pts = np.zeros((num_pts, 4), float)
            continue
        elif t[0] == 'DATA':
            data_started = True
            continue
        elif data_started:
            z = np.array(map(lambda x: float(x), t))
            pts[count,:] = z
            count += 1

    return pts

def loadLDRCamMap(frame_cloud_map):
    map_file = open(frame_cloud_map, 'r')
    clouds = []
    frame_folder, map_name = os.path.split(frame_cloud_map)
    frame_folder = frame_folder + '/' + map_name.split('.')[0] + '_frames'

    for line in map_file:
        ldr_file = line.rstrip().split(' ')[1]
        clouds.append(frame_folder + '/' + ldr_file)
    map_file.close()

    return clouds

def R_to_c_from_l_old(cam):
    # hard coded calibration parameters for now
    R_to_c_from_l = np.array([[0.0, -1.0, 0.0],
                              [0.0, 0.0, -1.0],
                              [1.0, 0.0, 0.0]])

    return R_to_c_from_l

def R_to_c_from_l(cam):
    # hard coded calibration parameters for now
    R_to_c_from_l = np.array([[0.0, -1.0, 0.0],
                              [0.0, 0.0, -1.0],
                              [1.0, 0.0, 0.0]])
    R_to_c_from_l = np.dot(cam['R_to_c_from_l_in_camera_frame'], R_to_c_from_l)

    return R_to_c_from_l

if __name__ == '__main__':
    import sys
    z = loadLDRCamMap(sys.argv[1])
    for x in z:
        print x
