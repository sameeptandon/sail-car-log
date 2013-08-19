# Usage: python two_lanes_display <path to video splits> <path to points in
# video frame> <path to points in other camera's frame>
import numpy as np
import pickle
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
    cam_num = int(vidname[-1]) - 1
    other_cam = (cam_num + 1) % 2
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    num_imgs_fwd = 125
    video_reader = VideoReader(video_filename)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    labels = loadmat(sys.argv[2])
    points = labels['points']

    labels_O = loadmat(sys.argv[3])
    points_O = labels_O['points']

    cam = pickle.load(open('cam_params.pickle', 'rb'))

    tr = GPSTransforms(gps_dat, cam[cam_num])
    tr_O = GPSTransforms(gps_dat, cam[other_cam])

    # probably have to change these
    pitch = 0.0
    height = 1.106
    p2 = 0.00
    R_to_c_from_i = cam[cam_num]['R_to_c_from_i']
    R_camera_pitch = euler_matrix(cam[cam_num]['rot_x'], cam[cam_num]['rot_y'],
            cam[cam_num]['rot_z'], 'sxyz')[0:3,0:3]
    R_to_c_from_i = dot(R_camera_pitch, R_to_c_from_i)
    Tc = np.eye(4)
    Tc[0:3, 0:3] = np.transpose(R_to_c_from_i)
    #Tc[1, 3] -= height
    Tc = np.array([[1, 0, 0, 0], [0, np.cos(pitch), -np.sin(pitch), -height], [0, np.sin(pitch), np.cos(pitch), 0], [0, 0, 0, 1]])

    Tc2 = np.eye(4) # check testTrackReverse for actual transformation value

    d = np.array([-0.91025106806, -0.01152806894, 0.01762668658])
    R_pitch = euler_matrix(deg2rad(0.5), 0,
            0, 'sxyz')[0:3,0:3]
    R = np.array([[ 0.99979991, -0.01643107, -0.01140847],
           [ 0.01654639,  0.9998122 ,  0.01008901],
                  [ 0.01124056, -0.01027576,  0.99988402]])

    #R = np.eye(3)
    T_O = np.eye(4)
    T_O[0:3,0:3] = np.dot(R_pitch, R)
    T_O[0:3, 3] = np.dot(R.transpose(), d)

    if cam_num == 1:
        T_O = np.linalg.inv(T_O)

    count = 0
    start = 0
    start_O = 0
    while True:
        (success, I) = video_reader.getNextFrame()
 
        if not success:
            break
        if count % 25 != 0:
            count += 1
            continue

        if count % 100 == 0:
            print count
        while start < points.shape[0] and points[start][4] < count:
            start += 1

        while start_O < points_O.shape[0] and points_O[start_O][4] < count:
            start_O += 1

        points_count = start
        while points_count < points.shape[0] and points[points_count][4] <= count + num_imgs_fwd:
            pt = points[points_count]
            Pos2 = np.linalg.solve(tr[count,:,:], (pt[0:4])) # will have to change to deal with Tc2

            pos2 = np.round(np.dot(cam[cam_num]['KK'], Pos2[0:3]) / Pos2[2])

            if pos2[1] > 3 and pos2[1] < 957 and pos2[0] > 3 and pos2[0] < 1277:
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 0] = 255
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 1] = 0
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 2] = 0
            points_count += 1


        points_count_O = start_O
        while points_count_O < points_O.shape[0] and points_O[points_count_O][4] <= count + num_imgs_fwd:
            pt = points_O[points_count_O]

            Pos2 = np.dot(T_O, np.linalg.solve(tr_O[count,:,:], pt[0:4]))

            pos2 = np.round(np.dot(cam[cam_num]['KK'], Pos2[0:3]) / Pos2[2])

            if pos2[1] > 3 and pos2[1] < 957 and pos2[0] > 3 and pos2[0] < 1277:
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 0] = 0
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 1] = 255
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 2] = 0
            points_count_O += 1

        count += 1
        I = imresize(I, (480, 640))
        imshow('video', I)
        key = waitKey(5)
        if key == ord('q'):
            break




