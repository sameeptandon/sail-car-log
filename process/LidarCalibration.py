from LidarTransforms import * 
import sys, os
from VideoReader import *
from CameraParams import * 
import cv2
from cv2 import imshow, waitKey
from numpy.linalg import norm
from ColorMap import *
from numpy import exp
from transformations import euler_matrix

def generateEdgeFilterKernels(): 
    kernels = []
    for x in range(3):
        for y in range(3):
            K = np.zeros((3,3))
            K[1,1] = 3.0
            K[x,y] = -3.0
            if (x != 1 and y != 1):
                kernels.append(K)
    return kernels

def processPointCloud(raw_pts): 
    # add rotational angle and distance to pts
    pts = np.zeros((raw_pts.shape[0], raw_pts.shape[1]+2))
    pts[:,:-2] = raw_pts
    pts[:,-2] = np.arctan2(pts[:,2], pts[:,1]) + np.pi
    pts[:,-1] = np.sum( pts[:, 0:3] ** 2, axis=1 ) 

    pts = pts[ pts[:,5].argsort() ] # sort on rotational angle
    pts = pts[ pts[:,4].argsort(kind='mergesort') ] # stable sort on laser num

    pts[0,3] = 0.0;
    pts[-1,3] = 0.0
    for idx in range(1,pts.shape[0]-1):
        if pts[idx,4] == pts[idx-1,4] and pts[idx,4] == pts[idx+1,4]: 
            pts[idx,3] = max(pts[idx-1,6] - pts[idx,6], 
                             pts[idx+1,6] - pts[idx,6],
                             0) ** (0.5)
        else:
            pts[idx,3] = 0.0
    return pts


if __name__ == '__main__': 
    video_filename = sys.argv[1] 
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    cam = getCameraParams()[cam_num - 1] 
    video_reader = VideoReader(video_filename,num_splits=1) # require only split_0 
    pts = loadLDR(sys.argv[2])
    pts = processPointCloud(pts)
    pts = pts[ pts[:, 3] > 0.5, :]

    width = 8
    # translate points in lidar frame to camera frame
    tx = -0.0
    ty = 0.1
    tz = -0.0
    rx = 0.0
    ry = 0.0
    rz = 0.0
    count = 0 
    while count < 10:
        (success, I) = video_reader.getNextFrame()
        count += 1
    # convert the image to grayscale
    I = cv2.cvtColor(I, cv2.COLOR_BGR2GRAY)
    imshow('displya', I)

    # run an edge filter
    kernels = generateEdgeFilterKernels()
    edges = cv2.filter2D(I, cv2.CV_8U, np.zeros((1,1)))
    
    for k in kernels: 
        edges = np.maximum(edges, np.abs(cv2.filter2D(I, cv2.CV_8U, k)))
   

    #edges [ edges < 20  ] = 0
    #edges [ edges >= 20 ] = 1
    # run distance transform
    #edges = cv2.distanceTransform(1-edges, cv.CV_DIST_L1, maskSize=3)
    #print edges[0]
    I = cv2.blur(edges, (15,15))
    raw_pts = array(pts[:, 0:3])
    raw_pts[:, 0] += tx
    raw_pts[:, 1] += ty
    raw_pts[:, 2] += tz
    R = euler_matrix(rx, ry, rz)[0:3,0:3]


    pts_wrt_cam = dot(R, dot(R_to_c_from_l(cam), raw_pts.transpose()))

    pix = np.around(np.dot(cam['KK'], np.divide(pts_wrt_cam[0:3,:], pts_wrt_cam[2, :])))
    pix = pix.astype(np.int32)
    mask = np.logical_and(True, pix[0,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[1,:] > 0 + width/2)
    mask = np.logical_and(mask, pix[0,:] < 1279 - width/2)
    mask = np.logical_and(mask, pix[1,:] < 959 - width/2)
    mask = np.logical_and(mask, pts_wrt_cam[2,:] > 0)

    dist_wrt_cam = np.sum( pts[mask, 0:3] ** 2, axis=1 ) ** (1.0/2)
    intensity = pts[mask, 3]
    #colors = heatColorMapFast(dist_wrt_cam, 0, np.max(dist_wrt_cam))
    #colors = heatColorMapFast(exp(-dist_wrt_cam/3), 0, 1)
    colors = heatColorMapFast(intensity, 0, 2)

    px = pix[1,mask]
    py = pix[0,mask]
    #I[px, py, :] = colors[0,:,:]
    I[px,py] = 255
    
    imshow('display', I)
    key = chr((waitKey() & 255))
