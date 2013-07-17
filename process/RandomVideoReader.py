from VideoReader import *
import cv, cv2
import os
import random

class RandomVideoReader(VideoReader):
  def initReader(self): 
    VideoReader.initReader(self)
    self.lengths = []
    self.order = []
    for capture in self.captures:
      length = int(capture.get(cv.CV_CAP_PROP_FRAME_COUNT))
      self.lengths.append(length)
      list_order = range(length)
      random.shuffle(list_order)
      self.order.append(list_order)

    self.cap_num = 0; 

  def getNextFrame(self):
    if self.framenum == self.lengths[self.cap_num]:
      self.framenum = 0
      self.cap_num += 1

    if self.cap_num >= len(self.captures):
      return (False, None)

    cap = self.captures[self.cap_num]
    frame_in_stream = self.order[self.cap_num][self.framenum]
    cap.set(cv.CV_CAP_PROP_POS_FRAMES, frame_in_stream)
    success, img = cap.read()
    self.framenum = self.framenum + 1;

    frame = self.cap_num + frame_in_stream*self.num_splits
    return (success, img, frame) 

  def playVideo(self):
    self.cap_num = 0
    while True:
      (success, img, frame) = self.getNextFrame()
      if success == False:
        break
      cv2.imshow("video", img)
      key = cv2.waitKey(5)
