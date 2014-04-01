#python LidarReprojection.py ../lidar/build/data/280/280S_a2.avi ../lidar/build/data/280/280S_a.map out.avi

from LidarTransforms import *
import sys, os
from VideoReader import *
from CameraParams import *
from cv2 import imshow, waitKey
import cv2
from numpy.linalg import norm
from ColorMap import *
from numpy import exp, linspace
from transformations import euler_matrix
from scipy.cluster.hierarchy import single
from scipy.spatial.distance import pdist
from scipy.interpolate import UnivariateSpline
import random
import colorsys

def getNextData(VideoReader, LDRFrameMap):
    for idx in range(5):
        (success, img) = VideoReader.getNextFrame()
        if not success:
            return None
    ldr_frame = loadLDR(LDRFrameMap[VideoReader.framenum])
    # print LDRFrameMap[VideoReader.framenum]
    return (success, img, ldr_frame)

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

def project_pts(pts, T, cam, width):
    raw_pts = array(pts[:, 0:3])
    raw_pts[:, 0] += T[0]
    raw_pts[:, 1] += T[1]
    raw_pts[:, 2] += T[2]
    R = euler_matrix(T[3], T[4], T[5])[0:3,0:3]

    pts_wrt_cam = dot(R, dot(R_to_c_from_l(cam), raw_pts.transpose()))

    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)

    return (pts[mask].transpose(), pix[:2, mask])

def get_lanes(pts, pix):
    pts = pts.transpose()
    pix = pix.transpose()
    lane_pts = pts[(pts[:, 3] > 60) & (pts[:, 0] > 0) & (pts[:, 2] < .5) & (abs(pts[:, 1]) < 5)]
    lane_pix = pix[(pts[:, 3] > 60) & (pts[:, 0] > 0) & (pts[:, 2] < .5) & (abs(pts[:, 1]) < 5)]

    clustered_lanes_pts = []
    clustered_lanes_pix = []
    if lane_pix.shape[0] > 0:
        clusters = hierarchical_clustering(lane_pix, 250)
        print [len(cluster) for cluster in clusters if len(cluster) > 1]
        for cluster in clusters:
            if len(cluster) > 1:
                pts_cluster = lane_pts[cluster]
                pix_cluster = lane_pix[cluster]

                horiz_groups = hierarchical_clustering(pts_cluster[:, :3], 2)

                clustered_lane_pts = np.array([np.median(pts_cluster[group], axis=0) for group in horiz_groups if len(group) > 0])
                clustered_lane_pix = np.array([np.median(pix_cluster[group], axis=0) for group in horiz_groups if len(group) > 0] ,dtype=np.int32)

                clustered_lanes_pts.append(clustered_lane_pts)
                clustered_lanes_pix.append(clustered_lane_pix)
            else:
                break

    return (clustered_lanes_pts, clustered_lanes_pix)

if __name__ == '__main__':
    video_filename = sys.argv[1]
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    cam = getCameraParams()[cam_num - 1]
    video_reader = VideoReader(video_filename,num_splits=1) # require only split_0 

    width = 9
    # translate points in lidar frame to camera frame

    T = (tx, ty, tz, rx, ry, rz) = \
            [-0.30493086,  0.41796525,  0.39775339, -0.0986506,   0.01598486,  0.00963721] #a2
            #[-0.30493086, -0.43203475, 0.49775339, -0.10865, 0.03598486, -0.01536279] #a1
    video_reader = VideoReader(video_filename)
    ldr_frame_map = loadLDRCamMap(sys.argv[2])
    
    count = 0
    while count < 10:
        (success, I, pts) = getNextData(video_reader, ldr_frame_map)
        count += 1
    
    # writer = cv2.VideoWriter(sys.argv[3], cv.CV_FOURCC('F','M','P','4'), 10.0, (1280,960))

    while True:
        (success, I, pts) = getNextData(video_reader, ldr_frame_map)
        if not success:
            break

        (proj_pts, proj_pix) = project_pts(pts, T, cam, width)

        (lane_pts, lane_pix) = get_lanes(proj_pts, proj_pix)
        
        if len(lane_pix) > 0:
            for lane in lane_pix:
                px = lane[:, 1].transpose()
                py = lane[:, 0].transpose()

                color = colorsys.hsv_to_rgb(random.random(), 1, 1)
                color = [int(c*255) for c in color]

                if px.shape > (0, 0):
                    I[px, py, :] = color
                    for i in xrange(0,width/2):
                        I[px+i,py, :] = color
                        I[px,py+i, :] = color
                        I[px-i,py, :] = color
                        I[px,py-i, :] = color

                if lane.shape[0] > 3:
                    s = UnivariateSpline(px, py)
                    xs = linspace(0, 960, 100).astype(np.int32)
                    ys = s(xs).astype(np.int32)
                    curve = np.column_stack((xs, ys)).astype(np.int32)
                    for c in curve:
                        x = c[0]
                        y = c[1]
                        if 0 < x < 960 and 0 < y < 1280:
                            I[c[0] ,c[1], :] = color

        # writer.write(I)
        # I = cv2.pyrDown(I)
        imshow('display', I)

        key = chr((waitKey(1) & 255))
        while key != ' ':
            key = chr((waitKey(10000) & 255))
