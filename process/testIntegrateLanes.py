# python testIntegrateLanes.py npz_data/ 280S_a2.avi npz_data/output_map_reflective.npz

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
        (-0.0, 0.33, 0.265, 0.041, 0.013, 0.014)
cR = euler_matrix(crx, cry, crz)[0:3,0:3]

start_fn = 0
num_fn = 10
step = 9*2

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

def hierarchical_clustering(pts, threshold, weights):
    distances = pdist(pts, w = weights)
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

        # Remove empty lists
        clusters = filter(None, clusters.values())
        # Inplace sort is slightly faster
        clusters.sort(key = lambda x: len(x), reverse = True)
        return clusters
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

    T_from_gps_to_l = [dot(T_from_i_to_l, np.linalg.inv(xform)) for xform in imu_transforms]
    T_from_l_to_gps = [np.linalg.inv(xform) for xform in T_from_gps_to_l]

    all_data = np.load(sys.argv[3])
    data = all_data['data']
    # data = data[data[:, 3] > 60]
    # np.savez_compressed('npz_data/output_map_reflective.npz', data=data)
    # print data

    delta = 15

    should_cluster = True

    count = 0
    while count < 300:
        video_reader.getNextFrame()
        stepVideo(video_reader, step)
        count += 1
    
    # writer = cv2.VideoWriter('out-alllanes.avi', cv.CV_FOURCC('F','M','P','4'), 10.0, (1280,960))

    while True:
        (success, I) = video_reader.getNextFrame()
        stepVideo(video_reader, step)

        all_pix = None

        frame_num = video_reader.framenum - step

        last_frame_num = delta*step + frame_num
        all_pts = data[data[:, 4] >= frame_num]
        all_pts = all_pts[all_pts[:, 4] < last_frame_num]
        filtered_pts = None
        for i in xrange(delta):
            t2 = i*step + frame_num
            pts_t2 = all_pts[all_pts[:, 4] == t2]

            if pts_t2.shape[0] > 0:
                pts_t2_h = np.vstack((pts_t2.transpose()[:3, :], np.ones((1, pts_t2.shape[0]))))
                pts_t1 = dot(T_from_gps_to_l[t2], pts_t2_h)

                # Filter points by their distance in the t1 reference frame
                pts_t2 = pts_t2[(pts_t1[2, :] < -1.5) & 
                                (pts_t1[2, :] > -2.5) &       # Z between -1.5 and -2.5
                                (abs(pts_t1[1, :]) < 2.2) &   # Y between -2.2 and 2.2 (use 10 for all lanes)
                                (abs(pts_t1[1, :]) > 1.2)     # Y outside -1.2 and 1.2
                                ]

                if pts_t2.shape[1] > 0:
                    filtered_pts = pts_t2 if filtered_pts == None else np.vstack((filtered_pts, pts_t2))

        # Convert to homogenous coordinates
        all_pts_h = np.vstack((filtered_pts.transpose()[:3, :], np.ones((1, filtered_pts.shape[0]))))

        clusters = hierarchical_clustering(all_pts_h.transpose(), 3, np.array([1, 10, 1, 0]))

        if should_cluster:
            cluster_list = [np.median(np.array(all_pts_h[:, cluster]) , axis=1) for cluster in clusters]
            all_pts_h = np.array(cluster_list).transpose()

        # Convert from point cloud to relative position of points from velodyne view        
        rel_all_pts = dot(T_from_gps_to_l[frame_num], all_pts_h)

        if rel_all_pts.shape[1] > 0:
            # Project the point cloud into the camera view
            pts_wrt_cam = rel_all_pts[:3, :]
            pts_wrt_cam[0, :] += ctx
            pts_wrt_cam[1, :] += cty
            pts_wrt_cam[2, :] += ctz
            pts_wrt_cam = dot(cR, dot(R_to_c_from_l(cam), pts_wrt_cam))
            (pix, mask) = cloudToPixels(cam, pts_wrt_cam)

            pix = pix[:2, mask]
            intensity = all_pts[mask, 3].astype(np.int32)

            pix = np.vstack((pix, intensity)).astype(np.int32)
            all_pix = pix if all_pix == None else np.hstack((all_pix, pix))


        px = all_pix[1, :]
        py = all_pix[0, :]
        intensties = all_pix[2, :]

        colors = heatColorMapFast(intensties, 0, 100).astype(np.int32)[0, :, :]

        if px.shape > (0, 0):
            I[px, py, :] = colors
            for i in xrange(3):
                I[px+i,py, :] = colors
                I[px,py+i, :] = colors
                I[px-i,py, :] = colors
                I[px,py-i, :] = colors

        cv2.imshow('display', I)
        # writer.write(I)

        print 'frame'
        key = chr((waitKey(1) & 255))
        # while key != ' ':
        #     key = chr((waitKey(2000000000) & 255))
