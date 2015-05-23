import os
import glob
import numpy as np
from scipy.ndimage.filters import convolve1d




def timeToSeconds(time):
  # converts time format of xx:xx:xx into seconds
  tokens = time.split(':')
  assert len(tokens)==3,'wrong time format'
  return float(tokens[0])*3600+float(tokens[1])*60+float(tokens[2])

def readTimeFile(filename, num_lines):
  times = np.zeros(num_lines)
  f = open(filename)
  cnt = 0
  for line in f:
    times[cnt] = timeToSeconds(line.rstrip().split(' ')[1])
    cnt+=1
  f.close()
  return times

def smoothData(arr, wind=51):
  filt = np.ones(wind) / wind
  return convolve1d(arr, filt, mode='nearest')

def integrateVelocity(v_arr, d0):
    return d0+ np.cumsum(v_arr)

def smoothDerivativeData(arr, wind=51):
    darr = arr[1:] - arr[:-1]
    darr = smoothData(darr, wind)
    return integrateVelocity(darr, arr[0])

def holdDerivativeData(arr, clipVal=0.1):
    darr = arr[1:] - arr[:-1]
    darr = np.clip(darr, -clipVal, clipVal)
    return integrateVelocity(darr,arr[0])


class GPSReader():
  def __init__(self, foldername):
    self.data = [ ]
    self.token_order = ['seconds', 'lat', 'long', 'height',
        'v_north', 'v_east', 'v_up', 'rot_y', 'rot_x',
        'azimuth', 'week'];
    self.kitti_token_order = ['lat', 'long', 'height','rot_x','rot_y','azimuth',
        'v_north', 'v_east', 'v_forward', 'v_left', 'v_up', 
        'ax', 'ay', 'az', 'af', 'al', 'au',
        'wx', 'wy', 'wz', 'wf', 'wl', 'wu',
        'pos_accuracy', 'vel_accuracy','navstat','numsats','posmode','velmode','orimode'];

    gps_files = sorted(list(glob.glob(os.path.join(foldername, 'data/*.txt'))))
    time_filename = os.path.join(foldername, 'timestamps.txt')
    time_f = open(time_filename)
    
    for filename in gps_files:
      f = open(filename);
      line = f.readline()
      f.close()
      l = line.rstrip()
      tokens = l.split(' ')
      time_line = time_f.readline()
      time_l = time_line.rstrip()
      time_tokens = time_l.split(' ')
      record = { }
      record['seconds'] = timeToSeconds(time_tokens[1])
      record['week'] = 1800 #dummy
      for idx in range(len(self.kitti_token_order)):
        record[self.kitti_token_order[idx]] = float(tokens[idx])
      # convert east-based, right handed yaw to north-based, left handed azimuth in our convention
      record[self.kitti_token_order[6]] = -record[self.kitti_token_order[6]]+np.pi/2.0
      self.data.append(record)
    time_f.close()
  def getData(self):
    return self.data;

  def getNumericData(self):
    arr = np.zeros([len(self.data), 11], np.float64);
    for t in range(len(self.data)):
      for j in range(11):
        arr[t,j] = self.data[t][self.token_order[j]]


    return arr;


