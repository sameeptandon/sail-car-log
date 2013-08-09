# usage:
# python test_video_reader.py <path to video splits> <output filename>

import sys
from VideoReader import VideoReader
import cv2, cv
import time

if __name__ == '__main__':
  reader = VideoReader(sys.argv[1])
  writer = cv2.VideoWriter(sys.argv[2], cv.CV_FOURCC('F','M','P','4'), 50.0, (1280,960))

  src = array([[499,597],[972,597],[1112,661],[448,678]], float32)
  dst = array([[320,320],[960,320],[960,640],[320,640]], float32)
  P = cv2.getPerspectiveTransform(src,dst)
  frames = 0
  lastTime = time.time()
  while True:
    (success, img) = reader.getNextFrame()
    if success == False:
      break
    
    O = findLanesConvolution(img, P=P)

    writer.write(O)
    frames += 1
    currentTime = time.time()
    if (currentTime - lastTime) > 1:
      print 'Encoding at', frames, 'Hz.'
      lastTime = currentTime
      frames = 0



