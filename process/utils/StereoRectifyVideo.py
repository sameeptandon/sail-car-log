import cv2
import sys
from StereoCompute import * 
from ArgParser import *
import os


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

    path, left_name = os.path.split(video_file_left) 
    path, right_name = os.path.split(video_file_right)

    left_name = path  + '/rect_' + left_name
    right_name = path + '/rect_' + right_name

    

    writer_left = cv2.VideoWriter(left_name, cv.CV_FOURCC('X','V', 'I', 'D'), 50.0, (1280,960))
    writer_right = cv2.VideoWriter(right_name, cv.CV_FOURCC('X','V', 'I', 'D'), 50.0, (1280,960))
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

        writer_left.write(imgRectL)
        writer_right.write(imgRectR)

