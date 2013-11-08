from VideoReader import *
import cv, cv2
import os
import pickle
import random

class WarpedVideoReader(VideoReader):
  """
  WarpedVideoReader is a subclass of VideoReader, which can take in a list
  of perspective matrix transformations and outputs frames in a random order
  while iterating through the perspective transformations.
  
  All frames will eventually be shown with all transformation matrices.

  setPerspectives must be called before getNextFrame or playVideo can be
  called
  """

  def setPerspectives(self, filename): 
    self.persps = pickle.load(open(filename, 'rb'))
    self.order = []
    self.length = int(self.captures[0].get(cv.CV_CAP_PROP_FRAME_COUNT))
    for P in self.persps:
      list_order = range(self.length)
      random.shuffle(list_order)
      self.order.append(list_order)

  def getNextFrame(self):
    if self.framenum >= self.length * len(self.persps):
      return (False, None, -1, None)

    p_number = self.framenum % len(self.persps)
    P = self.persps[p_number]
    frame_in_stream = self.order[p_number][self.framenum / len(self.persps)]
    cap = self.captures[0]
    cap.set(cv.CV_CAP_PROP_POS_FRAMES, frame_in_stream)
    success, img = cap.read()
    self.framenum = self.framenum + 1;

    frame = frame_in_stream*self.num_splits
    (rows, cols, channels) = img.shape
    img = cv2.warpPerspective(img, P, (cols, rows))
    return (success, img, frame, P) 

  def playVideo(self):
    self.cap_num = 0
    while True:
      (success, img, frame, P) = self.getNextFrame()
      if success == False:
        break
      print self.framenum
      cv2.imshow("video", img)
      #key = cv2.waitKey(5)
