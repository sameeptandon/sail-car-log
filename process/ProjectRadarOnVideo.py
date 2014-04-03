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

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])

    cam = GetQ50CameraParams()[cam_num - 1] 
    video_reader = VideoReader(args['video'])
    rdr_map = loadRDRCamMap(args['map'])
    
    writer = cv2.VideoWriter('radar_test.avi', cv.CV_FOURCC('X','V', 'I', 'D'),
                    10.0, (1280,960) )

    while True:
        for t in range(5):
            (success, I) = video_reader.getNextFrame()
        all_pix = None

        frame_num = video_reader.framenum

        radar_data = loadRDR(rdr_map[frame_num])[1]

        if radar_data.shape[0] > 0:
            print radar_data[:, 0:3]
            back_pts = transformLidarPointsToCameraPoints(radar_data[:,0:3], cam)
            
            front_pts = np.array(radar_data[:,0:3])
            front_pts[:,0] += radar_data[:,3]
            front_pts = transformLidarPointsToCameraPoints(front_pts, cam)
            
            right_pts = np.array(radar_data[:,0:3])
            right_pts[:,1] += radar_data[:,4] / 2
            right_pts = transformLidarPointsToCameraPoints(right_pts, cam)

            left_pts = np.array(radar_data[:,0:3])
            left_pts[:,1] -= radar_data[:,4] / 2
            left_pts = transformLidarPointsToCameraPoints(left_pts, cam)

            #print radar_data[:,5]



            # pts_wrt_camera_t_back = radar_data[:, 0:3] + cam['displacement_from_l_to_c_in_lidar_frame']
            # pts_wrt_camera_t_back = dot(R_to_c_from_l(cam), pts_wrt_camera_t_back.transpose())
            # # reproject camera_t points in camera frame
            (pix_front, mask) = cloudToPixels(cam, front_pts)
            (pix_back, mask) = cloudToPixels(cam, back_pts)
            (pix_left, mask) = cloudToPixels(cam, left_pts)
            (pix_right, mask) = cloudToPixels(cam, right_pts)

            for j in range(pix_front.shape[1]):
                cv2.line(I, tuple(pix_front[:,j]), tuple(pix_back[:,j]), (255,0,0))
                cv2.line(I, tuple(pix_right[:,j]), tuple(pix_left[:,j]), (255,255,0))


            # print pix[:, mask]

            # px = pix[1, mask]
            # py = pix[0, mask]

            # color = (255,255,0)
            # if px.shape > (0, 0):
            #     I[px, py, :] = color
            #     for i in xrange(3):
            #         I[px+i,py, :] = color
            #         I[px,py+i, :] = color
            #         I[px-i,py, :] = color
            #         I[px,py-i, :] = color

        cv2.imshow('display', I)
        # writer.write(I)

        key = chr((cv2.waitKey(1) & 255))