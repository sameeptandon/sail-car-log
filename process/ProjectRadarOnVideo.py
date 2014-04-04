from Q50_config import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
import numpy as np
import cv2
from ArgParser import *

(Rx,Ry,Rz) = (0, 0, -.015)

def cloudToPixels(cam, pts_wrt_cam): 

    width = 4
    (pix, J)  = cv2.projectPoints(pts_wrt_cam.transpose(), np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0]), cam['KK'], cam['distort'])

    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

def transformLidarPointsToCameraPoints(points, cam):
    pts_wrt_camera_t = points + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_camera_t = dot(R_to_c_from_l(cam), pts_wrt_camera_t.transpose())

    return pts_wrt_camera_t

def calibrateRadarPts(pts, Rxyz, Txyz=(3.17, 0.4, -1.64)):
    #T_pts[id] = [dist + 3.17, lat_dist + .4, -1.64, l, w]
    (Tx, Ty, Tz) = Txyz
    (Rx, Ry, Rz) = Rxyz
    R = euler_matrix(Rx, Ry, Rz)[0:3,0:3]

    pts[:, 0] += Tx
    pts[:, 1] += Ty
    pts[:, 2] += Tz
    return np.dot(R, pts.transpose()).transpose()

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])

    cam = GetQ50CameraParams()[cam_num - 1] 
    video_reader = VideoReader(args['video'])
    rdr_map = loadRDRCamMap(args['map'])
    
    writer = cv2.VideoWriter('radar_test.avi', cv.CV_FOURCC('X','V', 'I', 'D'),
                    50.0, (1280,960) )

    while True:
        for t in xrange(5):
            (success, I) = video_reader.getNextFrame()

        all_pix = None

        frame_num = video_reader.framenum

        radar_data = loadRDR(rdr_map[frame_num])[0]

        if radar_data.shape[0] > 0:
            radar_data[:, :3] = calibrateRadarPts(radar_data[:, :3], (Rx, Ry, Rz))

            # print radar_data[:, 0:3]
            back_pts = transformLidarPointsToCameraPoints(radar_data[:,0:3], cam)
            
            front_right_pts = np.array(radar_data[:,0:3])
            front_right_pts[:,0] += radar_data[:,3]
            front_right_pts[:,1] += radar_data[:,4] / 2.
            front_right_pts = transformLidarPointsToCameraPoints(front_right_pts, cam)
            
            front_left_pts = np.array(radar_data[:,0:3])
            front_left_pts[:,0] += radar_data[:,3]
            front_left_pts[:,1] -= radar_data[:,4] / 2.
            front_left_pts = transformLidarPointsToCameraPoints(front_left_pts, cam)

            right_pts = np.array(radar_data[:,0:3])
            right_pts[:,1] += radar_data[:,4] / 2.
            right_pts = transformLidarPointsToCameraPoints(right_pts, cam)

            left_pts = np.array(radar_data[:,0:3])
            left_pts[:,1] -= radar_data[:,4] / 2.
            left_pts = transformLidarPointsToCameraPoints(left_pts, cam)

            # reproject camera_t points in camera frame
            (pix_front_right, mask) = cloudToPixels(cam, front_right_pts)
            (pix_front_left, mask) = cloudToPixels(cam, front_left_pts)
            (pix_left, mask) = cloudToPixels(cam, left_pts)
            (pix_right, mask) = cloudToPixels(cam, right_pts)

            for j in range(pix_front_right.shape[1]):
                cv2.line(I, tuple(pix_front_right[:,j]), tuple(pix_front_left[:,j]), (255,0,0))
                cv2.line(I, tuple(pix_front_left[:,j]), tuple(pix_left[:,j]), (255,255,0))
                cv2.line(I, tuple(pix_left[:,j]), tuple(pix_right[:,j]), (255,255,0))
                cv2.line(I, tuple(pix_right[:,j]), tuple(pix_front_right[:,j]), (255,255,0))

        cv2.imshow('display', I)
        writer.write(I)

        print (Rx, Ry, Rz)

        key = chr((cv2.waitKey(1) & 255))
        if key == 'j':
            Rz += .001
        elif key == 'k':
            Rz -= .001