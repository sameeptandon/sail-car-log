# usage:
# python test_video_reader.py <path to video splits> <output filename>

import sys
from VideoReader import VideoReader
import cv2, cv
import time

if __name__ == '__main__':
  reader = VideoReader(sys.argv[1])
  writer = cv2.VideoWriter(sys.argv[2], cv.CV_FOURCC('F','M','P','4'), 50.0, (1280,960))

  frames = 0
  lastTime = time.time()
  while True:
    (success, img) = reader.getNextFrame()
    if success == False:
      break
    writer.write(img)
    frames += 1
    currentTime = time.time()
    if (currentTime - lastTime) > 1:
      print 'Encoding at', frames, 'Hz.'
      lastTime = currentTime
      frames = 0



