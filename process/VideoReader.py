import matplotlib.pylab as pp
import cv2, cv
import os

class VideoReader():
  def __init__(self, filename, in_splits=True, num_splits=10):
    assert(in_splits) # for now
    self.in_splits = in_splits;
    self.num_splits = num_splits;
    self.filename = filename;
    self.initReader();
    self.jump = 1 

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
    self.subsample = False

  def setSubsample(self, subs):
    self.subsample = subs

  def getNextFrame(self):
    self.jump = 10 if self.subsample else 1
    self.framenum = self.framenum + self.jump;
    success, img = self.captures[self.framenum % self.num_splits].read()
    return (success, img) 

  def setFrame(self, framenum):
      self.framenum = framenum;
      for j in range(1,self.num_splits+1, self.jump):
          capture_framenum = (framenum/10 + 1) if j-1 < framenum % 10 else framenum/10
          self.captures[j % self.num_splits].set(cv.CV_CAP_PROP_POS_FRAMES, capture_framenum)
          self.captures[j % self.num_splits].set(cv.CV_CAP_PROP_POS_FRAMES, capture_framenum)
          self.captures[j % self.num_splits].set(cv.CV_CAP_PROP_POS_FRAMES, capture_framenum)
          print self.captures[j % self.num_splits].get(cv.CV_CAP_PROP_FPS)
          #self.captures[j % self.num_splits].set(cv.CV_CAP_PROP_POS_MSEC, float(capture_framenum)/0.05)
          #a,b = self.captures[j % self.num_splits].read()
          print j, capture_framenum
          print self.captures[j % self.num_splits].get(cv.CV_CAP_PROP_POS_MSEC)
          print self.captures[j % self.num_splits].get(cv.CV_CAP_PROP_FRAME_COUNT)

  def playVideo(self):
    self.framenum = 0;
    while True:
      (success, img) = self.getNextFrame()
      if success == False:
        break;
      savename = '/scail/group/deeplearning/driving_data/stillimgs/280N_right_%d.png'%(self.framenum)
      img = img[:,:,::-1]
      print savename
      pp.imsave(savename, img)

#      cv2.imshow("video", img)
#      key = cv2.waitKey(5);
