#!/usr/bin/env python

'''
Performs OpenCV's StereoSGBM stereo matching. Usage py_stereo.py LeftImg RightImg
'''

from Q50_config import *
import sys, os
from GPSReader import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
import numpy as np
import scipy as sp
import cv2, cv
from ArgParser import *

def computeStereoRectify(params):
    imsize = (1280,960)
    cameraMatrix1 = params['cam'][0]['KK']
    distCoeffs1 = params['cam'][0]['distort']
    cameraMatrix2 = params['cam'][1]['KK']
    distCoeffs2 = params['cam'][1]['distort']

    (R1, R2, P1, P2, Q, size1, size2) = cv2.stereoRectify(
            cameraMatrix1, distCoeffs1, 
            cameraMatrix2, distCoeffs2, 
            imsize,
            params['cam'][0]['E_R'],
            params['cam'][0]['E_t'])
    
    map1x, map1y = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, imsize, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, imsize, cv2.CV_32FC1)

    return (R1, R2, P1, P2, Q, size1, size2, map1x, map1y, map2x, map2y)
 
def doStereo(imgL, imgR, params):
    """
    Parameters tuned for q50 car images.

    Parameters:
    minDisparity - Minimum possible disparity value. Normally, it is zero but sometimes rectification algorithms can shift images, so this parameter needs to be adjusted accordingly.
    numDisparities - Maximum disparity minus minimum disparity. The value is always greater than zero. In the current implementation, this parameter must be divisible by 16.
    SADWindowSize - Matched block size. It must be an odd number >=1. Normally, it should be somewhere in the 3..11 range.
    P1 - The first parameter controlling the disparity smoothness. See below.
    P2 - The second parameter controlling the disparity smoothness. The larger the values are, the smoother the disparity is. P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels. P2 is the penalty on the disparity change by more than 1 between neighbor pixels. The algorithm requires P2 > P1. See stereo_match.cpp sample where some reasonably good P1 and P2 values are shown (like 8*number_of_image_channels*SADWindowSize*SADWindowSize and 32*number_of_image_channels*SADWindowSize*SADWindowSize, respectively).
    disp12MaxDiff - Maximum allowed difference (in integer pixel units) in the left-right disparity check. Set it to a non-positive value to disable the check.
    preFilterCap - Truncation value for the prefiltered image pixels. The algorithm first computes x-derivative at each pixel and clips its value by [-preFilterCap, preFilterCap] interval. The result values are passed to the Birchfield-Tomasi pixel cost function.
    uniquenessRatio - Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct. Normally, a value within the 5-15 range is good enough.
    speckleWindowSize - Maximum size of smooth disparity regions to consider their noise speckles and invalidate. Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
    speckleRange - Maximum disparity variation within each connected component. If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16. Normally, 1 or 2 is good enough.
    fullDP - Set it to true to run the full-scale two-pass dynamic programming algorithm. It will consume O(W*H*numDisparities) bytes, which is large for 640x480 stereo and huge for HD-size pictures. By default, it is set to false.
    """
    imsize = (1280, 960) 
    (R1, R2, P1, P2, Q, size1, size2, map1x, map1y, map2x, map2y) = computeStereoRectify(params)

    imgRectL = cv2.remap(imgL, map1x, map1y, 
                interpolation=cv.CV_INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue = (0,0,0,0))
        
    imgRectR = cv2.remap(imgR, map2x, map2y, 
                interpolation=cv.CV_INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue = (0,0,0,0))
    """
    window_size = 1
    min_disp = 0
    num_disp = 64
    stereo = cv2.StereoSGBM(minDisparity = min_disp,
        numDisparities = num_disp,
        SADWindowSize = window_size,
        uniquenessRatio = 30,
        speckleWindowSize = 80,
        speckleRange = 1,
        disp12MaxDiff = 1,
        P1 = 8*3*window_size**2,
        P2 = 128*3*window_size**2,
        fullDP = True
    )
    """
    imgRectL = cv2.cvtColor(imgRectL, cv2.COLOR_RGB2GRAY)
    imgRectR = cv2.cvtColor(imgRectR, cv2.COLOR_RGB2GRAY)
    stereo = cv2.StereoBM(preset=cv.CV_STEREO_BM_NARROW,
            SADWindowSize=35)
    print 'computing stereo...'
    disp = stereo.compute(imgRectL, imgRectR).astype(np.float32) / 16.0
    return (disp, Q, R1, R2)


def siftStereo(imgL, imgR, params):
    (R1, R2, P1, P2, Q, size1, size2, map1x, map1y, map2x, map2y) = computeStereoRectify(params)

    imgRectL = cv2.remap(imgL, map1x, map1y, 
                interpolation=cv.CV_INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue = (0,0,0,0))
        
    imgRectR = cv2.remap(imgR, map2x, map2y, 
                interpolation=cv.CV_INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue = (0,0,0,0))

    grayL = cv2.cvtColor(imgRectL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(imgRectR, cv2.COLOR_BGR2GRAY)
    sift = cv2.SIFT()
    kpL, desL = sift.detectAndCompute(grayL,None)
    kpR, desR = sift.detectAndCompute(grayR,None)
    imgRectL = cv2.drawKeypoints(imgRectL, kpL, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('rect', imgRectL)

    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(desL,desR,k=2)

    good = []
    for m,n in matches:
        if m.distance > 0.5*n.distance:
            continue
        if np.abs(kpL[m.queryIdx].pt[1] - kpR[m.trainIdx].pt[1]) > 3:
            continue
        if kpL[m.queryIdx].pt[0] < kpR[m.trainIdx].pt[0]:
            continue

        good.append(m)
    matches = good                         

    h1, w1 = (960, 1280)
    h2, w2 = (960, 1280)
    disp = sp.zeros((max(h1,h2),w1), sp.float32)
    for m in matches:
        disp[kpR[m.trainIdx].pt[1], kpR[m.trainIdx].pt[0]] = (kpL[m.queryIdx].pt[0] - kpR[m.trainIdx].pt[0])

    disp = disp.astype(np.float32)
    view = sp.zeros((max(h1, h2), w1 + w2, 3), sp.uint8)
    view[:h1, :w1, 0] = grayL
    view[:h2, w1:, 0] = grayR
    view[:, :, 1] = view[:, :, 0]
    view[:, :, 2] = view[:, :, 0]

    for m in matches:
        # draw the keypoints
        # print m.queryIdx, m.trainIdx, m.distance
        color = tuple([sp.random.randint(0, 255) for _ in xrange(3)])
        cv2.line(view, (int(kpL[m.queryIdx].pt[0]), int(kpL[m.queryIdx].pt[1])) , (int(kpR[m.trainIdx].pt[0] + w1), int(kpR[m.trainIdx].pt[1])), color)

    #imgL = cv2.drawKeypoints(imgL, kp, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('imgLR', cv2.pyrDown(view))

    return (disp, Q, R1, R2)

"""
If necessary, the image can be reprojected to 3d to apply filters etc.

"""
def get3dPoints(disp, Q):
    """
    Q is the disparity to depth mapping calibrated with open CV code and Sameep's parameters
    """
    points = cv2.reprojectImageTo3D(disp, Q)

    #possible to do other filters such as removing objects beyond certain distance, height filters etc
    #mask = disp > disp.min()
    
    return points

if __name__ == '__main__':
    
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    params = args['params']
    assert(cam_num == 1)
    video_file_left = args['video']
    video_file_right = args['opposite_video']
    video_reader_left = VideoReader(video_file_left)
    video_reader_right = VideoReader(video_file_right)


    fnum = 0 
    while True:
        for t in range(10):
            (successL, imgL) = video_reader_left.getNextFrame()
            (successR, imgR) = video_reader_right.getNextFrame()
        

        """
        WRITEDIR = 'data/'
        imgL = cv2.cvtColor(imgL, cv2.COLOR_RGB2GRAY)
        imgR = cv2.cvtColor(imgR, cv2.COLOR_RGB2GRAY)
        cv2.imwrite(WRITEDIR+ 'left_%06d.png' % fnum, imgL)
        cv2.imwrite(WRITEDIR+ 'right_%06d.png' % fnum, imgR)
        fnum+=1;

        """   
        (disp, Q, R1, R2) = doStereo(imgL, imgR, params)
        points = get3dPoints(disp, Q)
        points = points[disp > 5, :]
        print points
        pts_wrt_lidar = np.dot(R_to_c_from_l(params['cam'][0]).transpose(), points.transpose())
        print pts_wrt_lidar.transpose()
        #imgL[disp > 10, :] = [255, 255, 0]
        cv2.imshow('imgL', cv2.pyrDown(imgL))
        cv2.imshow('imgR', cv2.pyrDown(imgR))
        cv2.imshow('disparity', cv2.pyrDown((disp)/64.0))
        cv2.waitKey(10);
        #(imgL, imgR) = doStereo(imgL, imgR, params)

        #bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        #matches = bf.match(desL, desR)
        #matches = sorted(matches, key = lambda x: x.distance)
        #imgLR = cv2.drawMatches(imgL,kpL,imgR,kpR,matches[:10],flags=2)
        """        
        (disp, Q, R1, R2) = siftStereo(imgL, imgR, params)        
        # visualization
        #disp = disp.astype(np.float32) / 16.0
        cv2.imshow('disp', cv2.pyrDown(disp))

        cv2.waitKey(1500)     #press q on keyboard to close
        """
    cv2.destroyAllWindows()
