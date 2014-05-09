from VideoReader import * 
from ArgParser import *
import sys, cv2

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    reader = VideoReader(args['video'])
    while True:
        (success, I) = reader.getNextFrame()
        if not success:
            break
        cv2.imshow('video', I)
        cv2.waitKey(1)
