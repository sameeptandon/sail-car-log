import cv2
import sys
from StereoCompute import * 
from ArgParser import *
import os
import numpy as np

def loadMatches(fname):
    arr = np.loadtxt(open(fname,'r'),delimiter=' ')
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

        pts = cv2.reprojectImageTo3D(disp, Q)
        pts = pts[disp > 5, :]

        print video_reader_left.framenum

        np.savez(sys.argv[3] + '/3d_' + str(video_reader_left.framenum), data=pts)

