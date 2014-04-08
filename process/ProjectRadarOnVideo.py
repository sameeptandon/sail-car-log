#python ProjectRadarOnVideo.py ../lidar/build/data/4-2-14-monterey/ 17N_b2.avi

from Q50_config import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from RadarTransforms import *
from LidarTransforms import *
from ColorMap import *
from transformations import euler_matrix
import numpy as np
import cv2
from ArgParser import *

def projectPoints(radar_data, args):
    """ Projects radar points into the camera's frame
        Args: radar_data, the output from loadRDR
              args, the output from parse_args
        Returns: A new numpy array with columns:
                    [dist, lat_dist, z(guess), l, w, rcs, rel_spd, id, x, y]
    """
    params = args['params']
    cam_num = args['cam_num']
    cam = params['cam'][cam_num - 1]

    radar_data[:, :3] = calibrateRadarPts(radar_data[:, :3], params['radar'])

    pts = radar_data[:, :3]
    pts_wrt_cam = pts + cam['displacement_from_l_to_c_in_lidar_frame']
    pts_wrt_cam = np.dot(R_to_c_from_l(cam), pts_wrt_cam.transpose())

    (pix, J)  = cv2.projectPoints(pts_wrt_cam.transpose(),
        np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0]),
        cam['KK'], cam['distort'])
    pix = pix.transpose()
    pix = np.around(pix[:, 0, :])
    pix = pix.astype(np.int32)

    # Filter the points to remove points that exist outside the video frame
    # This is not a problem for the radar because the cameras see everything 
    # the radar does
    # dist_sqr = np.sum(pts_wrt_cam[0:3, :] ** 2, axis = 0)
    # mask = (pix[0,:] > 0) & (pix[1,:] > 0) & \
    #     (pix[0,:] < 1279) & (pix[1,:] < 959) & \
    #     (pts_wrt_cam[2,:] > 0) & (dist_sqr > 3)

    # Outputs [dist, lat_dist, z(guess), l, w, rcs, rel_spd, id, x, y]
    radar_data_projected = np.hstack((radar_data, pix.transpose()))
    return radar_data_projected

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    params = args['params']

    video_reader = VideoReader(args['video'])
    rdr_map = loadRDRCamMap(args['map'])
    
    writer = cv2.VideoWriter('radar_test.avi', cv.CV_FOURCC('X','V', 'I', 'D'),
                    50.0, (1280,960) )

    while True:
        for t in xrange(5):
            (success, I) = video_reader.getNextFrame()

        frame_num = video_reader.framenum
        radar_data = loadRDR(rdr_map[frame_num])[0]

        if radar_data.shape[0] > 0:
            # Remove points that have a low radar cross-section
            mask = (radar_data[:, 5] > 5)
            # Remove points that are moving too fast (fixed objects)
            mask &= (radar_data[:, 6] > -20)
            radar_data = radar_data[mask]

        if radar_data.shape[0] > 0:
            front_right_pts = np.array(radar_data)
            front_right_pts[:,0] += radar_data[:,3]
            front_right_pts[:,1] += radar_data[:,4] / 2.
            
            front_left_pts = np.array(radar_data)
            front_left_pts[:,0] += radar_data[:,3]
            front_left_pts[:,1] -= radar_data[:,4] / 2.

            right_pts = np.array(radar_data)
            right_pts[:,1] += radar_data[:,4] / 2.

            left_pts = np.array(radar_data)
            left_pts[:,1] -= radar_data[:,4] / 2.

            # reproject camera_t points in camera frame
            front_right_proj = projectPoints(front_right_pts, args)
            front_left_proj = projectPoints(front_left_pts, args)
            left_proj = projectPoints(left_pts, args)
            right_proj = projectPoints(right_pts, args)

            for j in xrange(front_right_proj.shape[0]):
                fr = front_right_proj[j, 8:10].astype(np.int32)
                fl = front_left_proj[j, 8:10].astype(np.int32)
                bl = left_proj[j, 8:10].astype(np.int32)
                br = right_proj[j, 8:10].astype(np.int32)

                cv2.line(I, tuple(fr), tuple(fl), (255,255,0))
                cv2.line(I, tuple(fl), tuple(bl), (255,255,0))
                cv2.line(I, tuple(bl), tuple(br), (255,0,0))
                cv2.line(I, tuple(br), tuple(fr), (255,255,0))

                dist = radar_data[j, 0]
                rcs = radar_data[j, 5]
                spd = radar_data[j, 6]
                id = int(radar_data[j, 7])
                s = "%d: %d, %0.2f, %0.2f" % (id, rcs, dist, spd)
                cv2.putText(I, s, tuple(fr),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0,0,255), thickness=1)

        cv2.imshow('display', I)
        # writer.write(I)

        key = chr((cv2.waitKey(1) & 255))