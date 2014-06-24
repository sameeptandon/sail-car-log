from VideoReader import * 
from ArgParser import *
import sys, cv2, pickle

FRAME_STEP = 5
HISTORY_WINDOW = 30 
FUTURE_WINDOW = 20

def drawBorder(I, color, thickness): 
    cv2.rectangle(I, (0,0), (I.shape[1], I.shape[0]), color, thickness)

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    reader = VideoReader(args['video'])
    f = open(sys.argv[3], 'r')
    framenums = pickle.load(f)
    last_record = 0 
    while True:
        for p in range(FRAME_STEP):
            (success, I) = reader.getNextFrame()
        if not success:
            break

        if reader.framenum in framenums:
            drawBorder(I, (0,255,0), 10)
        cv2.imshow('video', I)
        key = chr(cv2.waitKey(50) & 255)

