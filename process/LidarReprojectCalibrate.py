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
        #cR = euler_matrix(crx, cry, crz)[0:3,0:3]
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
    #mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    #mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 1039 - width/2)
    mask = np.logical_and(mask, pix[0,:] < 2079 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

def lidarPtsToPixels(pts_wrt_lidar_t, imu_transforms_t, cam):
    # transform points from lidar_t to camera_t
    pts_wrt_camera_t = pts_wrt_lidar_t.transpose()[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = np.dot(cR, np.dot(R_to_c_from_l_old(0), 
            pts_wrt_camera_t.transpose()))

    pts_wrt_camera_t = np.vstack((pts_wrt_camera_t,
        np.ones((1,pts_wrt_camera_t.shape[1]))))
    pts_wrt_camera_t = dot(cam['E'], pts_wrt_camera_t)
    pts_wrt_camera_t = pts_wrt_camera_t[0:3,:]

    # reproject camera_t points in camera frame
    (pix, mask) = cloudToPixels(cam, pts_wrt_camera_t)

    return (pix, mask)

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    video_file = args['video']
    video_reader = VideoReader(video_file)
    params = args['params'] 
    cam = params['cam'][cam_num-1]
    gps_reader = GPSReader(args['gps_mark2'])
    gps_data = gps_reader.getNumericData()
    lidar_loader = LDRLoader(args['frames'])
    imu_transforms = IMUTransforms(gps_data)

    # parameter server
    thr = ThreadedServer(port)
    thr.setDaemon(True)
    thr.start()
    time.sleep(1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto('PARAMETER_REQUEST:'+str(port)+'\n', ('localhost', 2929))
    while not paramInit:
        time.sleep(1)

    video_reader.setFrame(int(sys.argv[3]))
    (success, orig) = video_reader.getNextFrame()
    while True:
        I = orig.copy()
        fnum = video_reader.framenum*2
        t = utc_from_gps_log(gps_data[fnum,:])
        data = lidar_loader.loadLDRWindow(t, 0.1)
        (pix, mask) = lidarPtsToPixels(data[:,0:3].transpose(), imu_transforms[fnum,:,:], cam); 
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
            cry += 0.005
        elif key == 'd':
            cry -= 0.005
        elif key == 'w':
            crx += 0.005
        elif key == 's':
            crx -= 0.005
        elif key == '+':
            crz += 0.005
        elif key == '_' or key == '-':
            crz -= 0.005
        else:
            continue
    
        print (crx, cry, crz)
        cR = euler_matrix(crx, cry, crz)[0:3,0:3]
        sock.sendto('PARAMETER_UPDATE:'+str(port)+':'+ParametersToString(rx,ry,rz,crx,cry,crz), ('localhost', 2929))
