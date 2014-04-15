from VideoReader import * 
from ArgParser import *
import sys, cv2

"""
usage: 
python PlayStereoVideo.py <path_to_data> <basename><camera_num>.avi
for example:
python PlayStereoVideo.py /scail/group/deeplearning/driving_data/q50_data/4-2-14-monterey/ 17N_b1.avi

under this command, reader1 will load 17N_b1.avi 
reader2 will load 17N_b2.avi

If you switch the command to python PlayStereoVideo.py /scail/group..../4-2-14-monterey/ 17N_b2.avi,
then you will have reader1 loading 17N_b2.avi and reader2 loading 17N_b1.avi. The point is that reader1 does not necessairly load the camera 1. 
"""

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    reader1 = VideoReader(args['video'])
    reader2 = VideoReader(args['opposite_video'])
    while True:
        (success1, I1) = reader1.getNextFrame()
        (success2, I2) = reader2.getNextFrame()
        if not success1 or not success2:
            break
        cv2.imshow('video1', cv2.pyrDown(I1))
        cv2.imshow('video2', cv2.pyrDown(I2))
        cv2.waitKey(5)
