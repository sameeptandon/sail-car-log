#usage
# python LidarReprojectCalibrate.py <dir-to-data> <basename> <start frame>

from Q50_config import *
import sys, os
from GPSReader import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
import numpy as np
import cv2
import socket
import time
from ArgParser import *
import SocketServer
import threading
import random
from MapBuilder import MapBuilder

global rx, ry, rz, crx, crz, R, cR, paramInit, port

paramInit = False
port = 3000 + int(random.random()*10000)

def ParametersToString(rx,ry,rz,crx,cry,crz):
    return "%f,%f,%f,%f,%f,%f\n" % (rx,ry,rz,crx,cry,crz)

class RequestHandler(SocketServer.BaseRequestHandler):
    def handle(self):
        global rx, ry, rz, crx, cry, crz, R, cR, paramInit

        data = self.request[0].strip()
        print data
        (rx,ry,rz,crx,cry,crz) = map(lambda x: float(x), data.split(','))

        R = euler_matrix(rx,ry,rz)[0:3,0:3].transpose()
        cR = euler_matrix(crx, cry, crz)[0:3,0:3]
        paramInit = True


class ThreadedServer(threading.Thread):
    def __init__(self, port):
        self.server = None
        self.port = port
        threading.Thread.__init__(self)
    def run(self):
        if self.server == None:
            address = ('localhost', self.port)
            self.server = SocketServer.UDPServer(address, RequestHandler)
        print 'starting server'
        self.server.serve_forever()


def cloudToPixels(cam, pts_wrt_cam):

    width = 4
    (pix, J)  = cv2.projectPoints(pts_wrt_cam.transpose(), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), cam['KK'], cam['distort'])
    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < cam['width'] - width/2 - 1)
    mask = np.logical_and(mask, pix[1,:] < cam['height'] - width/2 - 1)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

def lidarPtsToPixels(map_data, imu_transforms_t, T_from_i_to_l, cam):
    # Transform points back to imu_t
    pts_wrt_imu_0 = array(map_data[:, 0:3]).transpose()
    pts_wrt_imu_0 = np.vstack((pts_wrt_imu_0,
                               np.ones((1, pts_wrt_imu_0.shape[1]))))
    pts_wrt_imu_t = np.dot(np.linalg.inv(imu_transforms_t), pts_wrt_imu_0)
    #pts_wrt_imu_t = pts_wrt_imu_0

    # transform points from imu_t to lidar_t
    pts_wrt_lidar_t = np.dot(T_from_i_to_l, pts_wrt_imu_t)

    # transform points from lidar_t to camera_t

    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = np.dot(cR, np.dot(R_to_c_from_l_old(0),
            pts_wrt_camera_t.transpose()))

    pts_wrt_camera_t = np.vstack((pts_wrt_camera_t,
                                  np.ones((1, pts_wrt_camera_t.shape[1]))))
    pts_wrt_camera_t = dot(cam['E'], pts_wrt_camera_t)
    pts_wrt_camera_t = pts_wrt_camera_t[0:3, :]

    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    return (pix, mask)

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = args['cam_num']
    video_file = args['video']
    video_reader = VideoReader(video_file)
    params = args['params']
    cam = params['cam'][cam_num]

    gps_reader_mark1 = GPSReader(args['gps_mark1'])
    gps_data_mark1 = gps_reader_mark1.getNumericData()
    gps_reader_mark2 = GPSReader(args['gps_mark2'])
    gps_data_mark2 = gps_reader_mark2.getNumericData()

    imu_transforms_mark1 = IMUTransforms(gps_data_mark1)
    imu_transforms_mark2 = IMUTransforms(gps_data_mark2)
    gps_times_mark1 = utc_from_gps_log_all(gps_data_mark1)
    gps_times_mark2 = utc_from_gps_log_all(gps_data_mark2)
    builder = MapBuilder(args, 0, 999999, 0.1, 0.05)

    # parameter server
    thr = ThreadedServer(port)
    thr.setDaemon(True)
    thr.start()
    time.sleep(1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto('PARAMETER_REQUEST:'+str(port)+'\n', ('localhost', 2929))
    while not paramInit:
        time.sleep(1)

    print int(sys.argv[3])
    video_reader.setFrame(int(sys.argv[3]))
    (success, orig) = video_reader.getNextFrame()
    while True:
        I = orig.copy()
        fnum = video_reader.framenum
        print fnum
        T_from_l_to_i = np.eye(4)
        T_from_l_to_i[0:3,0:3] = R
        T_from_i_to_l = np.linalg.inv(T_from_l_to_i)
        t = gps_times_mark2[fnum]
        builder.start_time = t
        builder.end_time = t + 3.0 * 1e6
        builder.step_time = 0.5
        builder.T_from_l_to_i = T_from_l_to_i
        builder.T_from_i_to_l = T_from_i_to_l
        builder.buildMap(filters=['no-trees'])

        data, data_times = builder.getData()

        fnum_mark1 = bisect.bisect(gps_times_mark1, t) - 1

        (pix, mask) = lidarPtsToPixels(data,
                imu_transforms_mark1[fnum_mark1, :,:], T_from_i_to_l, cam)

        intensity = data[mask, 3]
        heat_colors = heatColorMapFast(intensity, 0, 100)
        for p in range(4):
            I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]
            I[pix[1,mask]+p, pix[0,mask], :] = heat_colors[0,:,:]
            I[pix[1,mask], pix[0,mask]+p, :] = heat_colors[0,:,:]

        cv2.imshow('vid', I)
        key = cv2.waitKey(10)
        if key == -1:
            continue
        key = chr(key & 255)
        if key == 'a':
            cry += 0.0005
        elif key == 'd':
            cry -= 0.0005
        elif key == 'w':
            crx += 0.0005
        elif key == 's':
            crx -= 0.0005
        elif key == '+':
            crz += 0.0005
        elif key == '_' or key == '-':
            crz -= 0.0005
        elif key == 'i':
            ry -= 0.005
        elif key == 'k':
            ry += 0.005
        elif key == 'u':
            rx += 0.005
        elif key == 'o':
            rx -= 0.005
        elif key == 'j':
            rz += 0.005
        elif key == 'l':
            rz -= 0.005
        else:
            continue

        print (rx, ry, rz, crx, cry, crz)
        R = euler_matrix(rx,ry,rz)[0:3,0:3].transpose()
        cR = euler_matrix(crx, cry, crz)[0:3,0:3]
        sock.sendto('PARAMETER_UPDATE:'+str(port)+':'+ParametersToString(rx,ry,rz,crx,cry,crz), ('localhost', 2929))
