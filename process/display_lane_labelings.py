import numpy as np
import sys
from cv2 import imread, imshow, resize, waitKey
from scipy.io import loadmat
from GPSReader import *
from GPSTransforms import *
from VideoReader import *

if __name__ == '__main__':
    video_filename = sys.argv[1]
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    num_imgs_fwd = 625
    video_reader = VideoReader(video_filename)
    gps_reader = GPSReader(gps_filename)
    gps_dat = gps_reader.getNumericData()

    labels = loadmat(sys.argv[2])
    xall = labels['xall'] - 8
    yall = labels['yall'] - 8


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
    while True:
        (success, I) = video_reader.getNextFrame()
        if not success:
            break

        if count % 5 != 0:
            count += 1
            continue

        if count % 100 == 0:
            print count
        for i in xrange(num_imgs_fwd):
            for j in xrange(2):
                x = xall[j, count + i]
                y = yall[j, count + i]
                Z = ((y-cam['cv'])*np.sin(pitch)*height + f*cos(pitch) * height)/(cos(pitch)*(y-cam['cv']) - f*sin(pitch))
                X = (cos(pitch)*Z-sin(pitch)*height)*(x-cam['cu'])/f
                Y = 0

                intermediate = np.linalg.solve(Tc, np.array([X, Y, Z, 1]))
                Pos = np.dot(tr[count + i,:,:], intermediate)
                Pos2 = np.linalg.solve(tr[count,:,:], Pos) # will have to change to deal with Tc2

                pos2 = np.round(np.dot(cam['KK'], Pos2[0:3]) / Pos2[2])

                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 0] = 255
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 1] = 0
                I[pos2[1]-3:pos2[1]+3, pos2[0]-3:pos2[0]+3, 2] = 0

        count += 1
        I = resize(I, (640, 480))
        imshow('video', I)
        key = waitKey(5)
        if key == ord('q'):
            break



