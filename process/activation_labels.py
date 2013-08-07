import cv
import cv2
import numpy as np
import sys
from GPSReader import *
from GPSTransforms import *
from VideoReader import *
from WGS84toENU import *
from generate_lane_labels import *
from scipy.ndimage.filters import convolve
from scipy.io import savemat
from skimage.morphology import label


if __name__ == '__main__':
    prev_x = -1*np.ones((2, 1))
    prev_y = -1*np.ones((2, 1))
    consec_borders = np.zeros((2, 1))

    points = np.empty((0, 7))

    remove_top = .75
    consec_sides = 5
    min_from_edge = 1
    min_from_side = 3

    skippedPoints = 1
    threshold = .05

    video_filename = sys.argv[1]
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    num_imgs_fwd = 25
    video_reader = VideoReader(video_filename)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    output_name = sys.argv[2]

    pitch = 0
    height = 1.106
    p2 = 0.00

    cam = {}
    cam['R_to_c_from_i'] = np.array([[-1, 0, 0],
                                     [0, 0, -1],
                                     [0, -1, 0]])

    if cam_num == 1:
        cam['rot_x'] = deg2rad(-0.8)  # better cam 1
        cam['rot_y'] = deg2rad(-0.5)
        cam['rot_z'] = deg2rad(-0.005)
        cam['t_x'] = -0.5
        cam['t_y'] = 1.1
        cam['t_z'] = 0.0
    elif cam_num == 2:
        cam['rot_x'] = deg2rad(-0.61)  # better cam 2
        cam['rot_y'] = deg2rad(0.2)
        cam['rot_z'] = deg2rad(0.0)
        cam['t_x'] = 0.5
        cam['t_y'] = 1.1
        cam['t_z'] = 0.0

    cam['fx'] = 2221.8
    cam['fy'] = 2233.7
    cam['cu'] = 623.7
    cam['cv'] = 445.7
    cam['KK'] = array([[cam['fx'], 0.0, cam['cu']],
                       [0.0, cam['fy'], cam['cv']],
                       [0.0, 0.0, 1.0]])

    Tc = np.array([[1, 0, 0, 0], [0, np.cos(pitch), -np.sin(pitch), -height], [0, np.sin(pitch), np.cos(pitch), 0], [0, 0, 0, 1]])
    f = (cam['fx'] + cam['fy']) / 2

    tr = GPSTransforms(gps_dat, cam)
    src = np.array([[499,597],[972,597],[1112,661],[448,678]], np.float32) / 4
    dst = np.array([[320,320],[960,320],[960,640],[320,640]], np.float32) / 4
    P = cv2.getPerspectiveTransform(src, dst)

    lastCols = [None, None]

    edge_size = 100
    count = 0
    ratio = 4
    while True:
        (success, I) = video_reader.getNextFrame()

        if success is False:
            break

        if count % 100 == 0:
            print count
            savemat(output_name, {'points': points})

        #if count == 5000:
        #    break

        imsize = (320,240)
        I = cv2.resize(I, imsize)
        (O, lastCols) = findLanes(I, (imsize[1], imsize[0]), lastCols, P)

        O_bin = O[:,:,2] > 0
        labels = label(O_bin, 8, 0)
        num = np.amax(labels)
        if num < 0:
            count += 1
            continue
        new_points = np.zeros((0,3))

        for i in xrange(0, num+1):
            tuple = np.where(labels == i)
            x = tuple[1] * ratio + ratio/2
            y = tuple[0] * ratio + ratio/2
            if x.size > 0:
                avg = np.mean(np.vstack([x, y]).transpose(), axis=0)
                avg = np.hstack([avg, x.size])
                new_points = np.append(new_points, avg.reshape((1,3)), axis=0)

        if True:    
            x = new_points[:, 0]
            y = new_points[:, 1]
            left_lane = x < 640

            Z = ((y-cam['cv'])*np.sin(pitch)*height + f*cos(pitch) * height)/(cos(pitch)*(y-cam['cv']) - f*sin(pitch))
            X = (cos(pitch)*Z-sin(pitch)*height)*(x-cam['cu'])/f
            Y = np.zeros((x.shape[0], 1))

            dirs = np.array([X, Y, Z])
            dirs = np.append(dirs, np.ones((1, dirs.shape[1])), 0)
            intermediate = np.linalg.solve(Tc, dirs)
            Pos = np.dot(tr[count, :, :], intermediate)
            Pos = np.append(Pos, count*np.ones((1, Pos.shape[1])), 0)
            Pos = np.append(Pos, new_points[:, 2].reshape((1, Pos.shape[1])), 0)
            Pos = np.append(Pos, left_lane.reshape((1, Pos.shape[1])), 0)
            points = np.append(points, Pos.transpose(), 0)

        count += 1
