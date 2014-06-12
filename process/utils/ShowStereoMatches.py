import cv2
import sys
from StereoCompute import * 
from ArgParser import *
import os
import numpy as np

def loadMatches(fname):
    arr = np.loadtxt(open(fname,'r'),delimiter=' ')
    print arr
    return arr


if __name__ == '__main__':
    
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    params = args['params']
    
    assert(cam_num == 1)
    video_file_left = args['video']
    video_file_right = args['opposite_video']
    video_reader_left = VideoReader(video_file_left)
    video_reader_right = VideoReader(video_file_right)

    (R1, R2, P1, P2, Q, size1, size2, map1x, map1y, map2x, map2y) = computeStereoRectify(args['params'])

    while True:
        (successL, imgL) = video_reader_left.getNextFrame()
        (successR, imgR) = video_reader_right.getNextFrame()

        if not successL or not successR:
            break

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
        matches = loadMatches(sys.argv[3] + '/stereo_' + str(video_reader_left.framenum) + '.txt')

        good = [ ]
        for m in matches:
            if m[0] < m[2]:
                continue
            good.append(m)

        matches = good

        h1, w1 = (960, 1280)
        h2, w2 = (960, 1280)
        disp = sp.zeros((max(h1,h2),w1), sp.float32)
        for m in matches:
            if m[0] < m[2]:
                continue
            disp[m[1], m[0]] = m[0] - m[2]

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
            cv2.line(view, (int(m[0]), int(m[1])) , (int(m[2] + w1), int(m[3])), color)

        #cv2.imshow('disp', disp)
        #cv2.waitKey(5)
        cv2.imshow('view', cv2.pyrDown(view))
        cv2.waitKey(10)
