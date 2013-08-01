import numpy as np
import sys
from cv2 import imread, imshow, resize, waitKey
from scipy.io import loadmat, savemat
from scipy.misc import imresize
from scipy.ndimage import gaussian_filter
from scipy.ndimage.morphology import *
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
    num_imgs_fwd = 50
    video_reader = VideoReader(video_filename)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    labels = loadmat(sys.argv[2])
    points = labels['points']

    output_name = sys.argv[3]

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
    end = 0
    scale = 4

    score = np.zeros((points.shape[0],))
    score_thresh = 0.25
    while True:
        (success, I) = video_reader.getNextFrame()
 
        if not success:
            break

        I = imresize(I, (240, 320))
        O = findLanes(I, (240, 320))

        response = O < 0.25
        score_O = distance_transform_edt(response)
        m = np.max(np.max(score_O))
        score_O = score_O / m
        score_O = np.exp(-25*score_O)
        #imshow('video', O)
        #waitKey()
        #score_O = gaussian_filter(O, sigma=7)
        #score_O = O

        if count % 10 == 0:
            print count

        if count % 100 == 0 and count > 0:
            savemat(output_name, {'score': score})
        while start < points.shape[0] and points[start,4] < count:
            start += 1
        while end < points.shape[0] and points[end,4] < count + num_imgs_fwd:
            end += 1

        if start == end:
            break

        pts = points[start:end]
        tr_inv = np.linalg.inv(tr[count,:,:])
        Pos2 = np.dot(tr_inv, pts[:,0:4].transpose())

        pos2 = np.around(np.dot(cam['KK'], np.divide(Pos2[0:3, :], Pos2[2, :])))
        y = pos2[1, :].astype(int)
        y = y / scale
        y[y < 0] = 0
        y[y > 239] = 239
        x = pos2[0, :].astype(int)
        x = x / scale
        x[x < 0] = 0
        x[x > 319] = 319
        score[start:end] = score[start:end] + score_O[y, x]

        """
        points_count = start
        while points_count < points.shape[0] and points[points_count][2] <= count + num_imgs_fwd:
            pt = points[points_count]
            x = pt[0]
            y = pt[1]
            Z = ((y-cam['cv'])*np.sin(pitch)*height + f*cos(pitch) * height)/(cos(pitch)*(y-cam['cv']) - f*sin(pitch))
            X = (cos(pitch)*Z-sin(pitch)*height)*(x-cam['cu'])/f
            Y = 0

            intermediate = np.linalg.solve(Tc, np.array([X, Y, Z, 1]))
            Pos = np.dot(tr[pt[2],:,:], intermediate)
            Pos2 = np.linalg.solve(tr[count,:,:], Pos) # will have to change to deal with Tc2

            pos2 = np.round(np.dot(cam['KK'], Pos2[0:3]) / Pos2[2])

            y = min(959, max(0, pos2[1]))
            x = min(1279, max(0, pos2[0]))
            score[points_count] += score_O[y, x]
            points_count += 1
        """
        count += 1

    stable_points = np.zeros((0, 3))
    for i in xrange(score.shape[0]):
        if score[i] > score_thresh:
            stable_points = np.append(stable_points, points[i])

    savemat(output_name, {'stable_points': stable_points, 'score': score})




