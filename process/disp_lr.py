import numpy as np
import sys
from cv2 import imread, imshow, resize, waitKey
from scipy.misc import imresize
from scipy.io import loadmat
from GPSReader import *
from GPSTransforms import *
from generate_lane_labels import *
from VideoReader import *

if __name__ == '__main__':
    video_filename = sys.argv[1]
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    num_imgs_fwd = 125
    video_reader = VideoReader(video_filename)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    labels = loadmat(sys.argv[2])
    left_points = labels['left']
    right_points = labels['right']

    cam = { }
    cam['R_to_c_from_i'] = array([[-1, 0, 0], \
                         [0, 0, -1], \
                         [0, -1, 0]]);

    if cam_num == 1:
        cam['rot_x'] = deg2rad(-0.8); # better cam 1
        cam['rot_y'] = deg2rad(-0.5);
        cam['rot_z'] = deg2rad(-0.005);
        cam['t_x'] = -0.5;
        cam['t_y'] = 1.1;
        cam['t_z'] = 0.0;
    elif cam_num == 2:
        cam['rot_x'] = deg2rad(-0.61); # better cam 2 
        cam['rot_y'] = deg2rad(0.2);
        cam['rot_z'] = deg2rad(0.0);
        cam['t_x'] = 0.5;
        cam['t_y'] = 1.1;
        cam['t_z'] = 0.0;

    cam['fx'] = 2221.8
    cam['fy'] = 2233.7
    cam['cu'] = 623.7
    cam['cv'] = 445.7
    cam['KK'] = array([[cam['fx'], 0.0, cam['cu']], \
                     [0.0, cam['fy'], cam['cv']], \
                     [0.0, 0.0, 1.0]]);

    tr = GPSTransforms(gps_dat, cam)

    # probably have to change these
    pitch = 0.0
    height = 1.106
    p2 = 0.00
    f = (cam['fx'] + cam['fy']) / 2
    R_to_c_from_i = cam['R_to_c_from_i']
    R_camera_pitch = euler_matrix(cam['rot_x'], cam['rot_y'],\
            cam['rot_z'], 'sxyz')[0:3,0:3]
    R_to_c_from_i = dot(R_camera_pitch, R_to_c_from_i)
    Tc = np.eye(4)
    Tc[0:3, 0:3] = np.transpose(R_to_c_from_i)
    #Tc[1, 3] -= height
    Tc = np.array([[1, 0, 0, 0], [0, np.cos(pitch), -np.sin(pitch), -height], [0, np.sin(pitch), np.cos(pitch), 0], [0, 0, 0, 1]])

    Tc2 = np.eye(4) # check testTrackReverse for actual transformation value

    count = 0
    start = 0
    while True:
        (success, I) = video_reader.getNextFrame()

        if not success:
            break
        if count % 25 != 0:
            count += 1
            continue

        if count > left_points.shape[0] or count > right_points.shape[0]:
            break

        start = count
        for points_count in xrange(start, min(left_points.shape[0], start+num_imgs_fwd)):
            pt = left_points[points_count]
            if pt[5] == 1:
                Pos2 = np.linalg.solve(tr[count,:,:], pt[0:4]) # will have to change to deal with Tc2

                pos2 = np.round(np.dot(cam['KK'], Pos2[0:3]) / Pos2[2])

                if pos2[1] > 3 and pos2[1] < 957 and pos2[0] > 3 and pos2[0] < 1277:
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 0] = 255
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 1] = 0
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 2] = 0

            pt = right_points[points_count]
            if pt[5] == 1:
                Pos2 = np.linalg.solve(tr[count,:,:], pt[0:4]) # will have to change to deal with Tc2

                pos2 = np.round(np.dot(cam['KK'], Pos2[0:3]) / Pos2[2])

                if pos2[1] > 3 and pos2[1] < 957 and pos2[0] > 3 and pos2[0] < 1277:
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 0] = 0
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 1] = 255
                    I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 2] = 0

        count += 1
        I = imresize(I, (480, 640))
        imshow('video', I)
        key = waitKey()
        if key == ord('q'):
            break




