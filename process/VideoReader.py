import matplotlib.pylab as pp
import cv2, cv
import os
from glob import glob

class VideoReader(object):
  def __new__(cls, filename, in_splits=True, num_splits=10):
    if '.avi' in filename:
      return AVIVideoReader(filename, in_splits, num_splits)
    else:
      return JPEGVideoReader(filename)

class AVIVideoReader:
  def __init__(self, filename, in_splits=True, num_splits=10):
    assert(in_splits) # for now
    self.in_splits = in_splits
    self.num_splits = num_splits
    self.filename = filename
    self.initReader()
    self.jump = 1

  def initReader(self):
    path, basename = os.path.split(self.filename)
    if path == '':
      path = './'
    self.names = [ ]
    for j in range(self.num_splits):
      self.names.append(path + '/' + 'split_' + str(j) + '_' + basename)

    self.captures = list()
    self.frame_counts = list()
    for j in range(self.num_splits):
      self.captures.append(cv2.VideoCapture(self.names[j]))
        # NOTE CV_CAP_PROP_FRAME_COUNT is known to sometimes be inaccurate
        # -> may need to use ffmpeg to estimate count
      self.frame_counts.append(self.captures[-1].get(cv.CV_CAP_PROP_FRAME_COUNT))
    self.framenum = 0;
    self.total_frame_count = sum(self.frame_counts)
    self.subsample = False

  def setSubsample(self, subs):
    self.subsample = subs

  def getNextFrame(self):
    self.jump = 10 if self.subsample else 1
    self.framenum = self.framenum + self.jump;
    success, img = self.captures[self.framenum % self.num_splits].read()
    return (success, img)

  def setFrame(self, framenum, verbose=False):
      self.framenum = framenum;
      for j in range(1,self.num_splits+1, self.jump):
          capture_framenum = (framenum/10 + 1) if j-1 < framenum % 10 else framenum/10
          self.captures[j % self.num_splits].set(cv.CV_CAP_PROP_POS_FRAMES, capture_framenum)
          #self.captures[j % self.num_splits].set(cv.CV_CAP_PROP_POS_MSEC, float(capture_framenum)/0.05)
          #a,b = self.captures[j % self.num_splits].read()
          if (verbose):
              print self.captures[j % self.num_splits].get(cv.CV_CAP_PROP_FPS)
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

class JPEGVideoReader:
  def __init__(self, frame_folder):
    self.framenum = 0
    self.total_frame_count = len(glob(frame_folder + '/*.jpg'))
    self.captures = sorted(glob(frame_folder + '/*'), key=lambda x:
                           int(os.path.basename(x).split('.')[0]))

  def getNextFrame(self):
    self.framenum += 1
    return self.getFrame(self.framenum)

  def getFrame(self, framenum):
    self.framenum = framenum
    if self.framenum < self.total_frame_count:
      return (True, cv2.imread(self.captures[self.framenum - 1]))
    else:
      return (False, None)

  def setFrame(self, framenum):
    self.framenum = framenum
