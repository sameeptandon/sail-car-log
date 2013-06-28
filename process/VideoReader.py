import cv2
import os

class VideoReader():
  def __init__(self, filename, in_splits=True, num_splits=10):
    assert(in_splits) # for now
    self.in_splits = in_splits;
    self.num_splits = num_splits;
    self.filename = filename;
    self.initReader();
    self.playVideo()

  def initReader(self): 
    path, basename = os.path.split(self.filename)
    if path == '':
      path = './'
    self.names = [ ]
    for j in range(self.num_splits):
      self.names.append(path + '/' + 'split_' + str(j) + '_' + basename)

    self.captures = [ ]
    for j in range(self.num_splits):
      self.captures.append(cv2.VideoCapture(self.names[j]))
    self.framenum = 0; 

  def getNextFrame(self):
    self.framenum = self.framenum + 1;
    success, img = self.captures[self.framenum % self.num_splits].read()
    return (success, img) 

  def playVideo(self):
    self.framenum = 0;
    while True:
      (success, img) = self.getNextFrame()
      if success == False:
        break;
      cv2.imshow("video", img)
      key = cv2.waitKey(5);


