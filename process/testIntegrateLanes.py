from Q50_config import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from transformations import euler_matrix
import numpy as np
from ColorMap import *
import copy
from CameraParams import * 
from ArgParser import *
import colorsys
from cv2 import imshow, waitKey
import cv2
from scipy.cluster.hierarchy import single
from scipy.spatial.distance import pdist

#CAMERA 2

(rx,ry,rz) = (0,-0.015,-0.045)
R = euler_matrix(rx,ry,rz)[0:3,0:3]
T_from_l_to_i = np.eye(4)
T_from_l_to_i[0:3,0:3] = R.transpose()
T_from_i_to_l = np.linalg.inv(T_from_l_to_i)

(ctx, cty, ctz, crx, cry, crz) = \
        (-0.0, 0.31, 0.15, 0.049, 0.016, 0.014)
cR = euler_matrix(crx, cry, crz)[0:3,0:3]

start_fn = 0
num_fn = 10
step = 9

def stepVideo(video_reader, step):
    if step == 1: 
        return None
    for t in range(step-1):
        (success, I) = video_reader.getNextFrame()
    return success

def cloudToPixels(cam, pts_wrt_cam): 

    width = 4
    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)
    dist_sqr = np.sum( pts_wrt_cam[0:3, :] ** 2, axis = 0)
    mask = np.logical_and(mask, dist_sqr > 3)

    return (pix, mask)

def hierarchical_clustering(pts, threshold):
    distances = pdist(pts)
    if distances.shape[0] > 0:
        links = single(distances)
        numLeafs = pts.shape[0]
        clusters = {i: [i] for i in xrange(numLeafs)}
        for i in xrange(numLeafs - 1):
            if links[i, 2] > threshold:
                break
            idx1 = int(links[i, 0] + 0.0001)
            idx2 = int(links[i, 1] + 0.0001)
            newIdx = i + numLeafs
            clusters[newIdx] = clusters[idx1] + clusters[idx2]
            clusters[idx1] = []
            clusters[idx2] = []
        return sorted(clusters.values(),lambda x,y: cmp(len(y), len(x)))
    else:
        return [[]]

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    gps_filename = args['gps']

    cam = getCameraParams()[cam_num - 1] 
    video_reader = VideoReader(args['video'])
    gps_reader = GPSReader(gps_filename)
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    # inv_imu_transforms = [np.linalg.inv(xform) for xform in imu_transforms]
    T_from_gps_to_l = [dot(T_from_i_to_l, np.linalg.inv(xform)) for xform in imu_transforms]

    all_data = np.load(sys.argv[3])
    data = all_data['data']
    
    delta = 25

    count = 0
    while count < 600:
        video_reader.getNextFrame()
        stepVideo(video_reader, step)
        count += 1
    
    writer = cv2.VideoWriter('out3.avi', cv.CV_FOURCC('F','M','P','4'), 10.0, (1280,960))

    while True:
        pts = data[data[:, 4] < (video_reader.framenum + step * delta)]
        pts = pts[pts[:, 4] >= (video_reader.framenum)]
        #Lane filter

        (success, I) = video_reader.getNextFrame()
        stepVideo(video_reader, step)

        all_pix = None

        frame_num = video_reader.framenum - step
        # initial_rel_gps_inv = np.linalg.inv(imu_transforms[frame_num])
        for i in xrange(delta):
            cur_frame_num = i*step + frame_num

            cur_pts = pts[pts[:, 4] == cur_frame_num]
            # Convert to homogenous coordinates
            cur_pts_h = np.vstack((cur_pts.transpose()[:3, :], np.ones((1, cur_pts.shape[0]))))

            # Convert from point cloud to relative position of points from velodyne view        
            rel_cur_pts = dot(T_from_gps_to_l[cur_frame_num], cur_pts_h)
            rel_cur_pts = rel_cur_pts[:, (cur_pts[:, 3] > 60) &
                                         (rel_cur_pts[0, :] > 0) &
                                         (rel_cur_pts[2, :] < -1.2) &
                                         (abs(rel_cur_pts[1, :]) < 4)]

            if rel_cur_pts.shape[1] > 0:
                # Project the point cloud into the camera view
                pts_wrt_cam = rel_cur_pts[:3, :]
                pts_wrt_cam[0, :] += ctx
                pts_wrt_cam[1, :] += cty
                pts_wrt_cam[2, :] += ctz
                pts_wrt_cam = dot(cR, dot(R_to_c_from_l(cam), pts_wrt_cam))
                (pix, mask) = cloudToPixels(cam, pts_wrt_cam)
                
                pix = pix[:2, mask]
                intensity = cur_pts[mask, 3].astype(np.int32)
                
                pix = np.vstack((pix, intensity)).astype(np.int32)
                all_pix = pix if all_pix == None else np.hstack((all_pix, pix))

        horiz_groups = hierarchical_clustering(all_pix[:2, :].transpose(), 30)

        all_pix_grouped = np.array([np.median(all_pix[:, group], axis=1) for group in horiz_groups if len(group) > 0] ,dtype=np.int32)
        all_pix_grouped = all_pix_grouped.transpose()

        vert_groups = hierarchical_clustering(all_pix_grouped[:2, :].transpose(), 100)

        for i in xrange(2):
            lane = np.array(all_pix_grouped[:, vert_groups[i]], dtype=np.int32)
            z = np.polyfit(lane[0, :].transpose(), lane[1, :].transpose(), 1)
            f = np.poly1d(z)
            x = np.linspace(0, 1280, 200)
            lane_pts = np.vstack((x, f(x))).transpose()
            lane_pts = lane_pts[lane_pts[:,1] > 0]
            lane_pts = lane_pts[lane_pts[:,1] > 480]
            if np.mean(lane[0, :]) < 640:
                color = (0, 0, 255)
            else:
                color = (0, 255, 0)
            cv2.polylines(I, np.int32([lane_pts]), False, color, thickness=2)

        px = all_pix_grouped[1, :]
        py = all_pix_grouped[0, :]
        intensties = all_pix_grouped[2, :]

        colors = heatColorMapFast(intensties, 0, 100).astype(np.int32)[0, :, :]

        if px.shape > (0, 0):
            I[px, py, :] = colors
            for i in xrange(3):
                I[px+i,py, :] = colors
                I[px,py+i, :] = colors
                I[px-i,py, :] = colors
                I[px,py-i, :] = colors

        # cv2.imshow('display', I)
        writer.write(I)
        print 'frame'
        key = chr((waitKey(1) & 255))
        # while key != ' ':
        #     key = chr((waitKey(10000) & 255))
