# usage:
# python test_video_reader.py <path to video splits> <output filename>

import sys
from VideoReader import VideoReader
import cv2, cv
import time
from numpy import float32, array, uint8
from generate_lane_labels import *

if __name__ == '__main__':
  reader = VideoReader(sys.argv[1])

  imsize = (320,240)
  writer = cv2.VideoWriter(sys.argv[2], cv.CV_FOURCC('F','M','P','4'), 50.0, imsize)

  src = array([[499,597],[972,597],[1112,661],[448,678]], float32) / 4
  dst = array([[320,320],[960,320],[960,640],[320,640]], float32) / 4
  P = cv2.getPerspectiveTransform(src,dst)
  frames = 0
  lastTime = time.time()
  while True:
    (success, img) = reader.getNextFrame()
    if success == False:
      break
    if frames > 5000:
      break
    
    img = cv2.resize(img, imsize)
    
    (O, lastCols,asdf2) = findLanesConvolution(img, P=P, origSize=(imsize[1], imsize[0]))

    O = O.astype(uint8)
    cv2.imshow('img', cv2.warpPerspective(O, P, imsize))
    cv2.waitKey(5)

    writer.write(O)
    frames += 1
    currentTime = time.time()
    if (currentTime - lastTime) > 1:
      print 'Encoding at', frames, 'Hz.'
      lastTime = currentTime
      frames = 0



