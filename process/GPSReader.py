import numpy as np
from scipy.ndimage.filters import convolve1d

def smoothData(arr, wind=51):
  filt = np.ones(wind) / wind
  return convolve1d(arr, filt, mode='nearest')

class GPSReader():
  def __init__(self, filename):
    self.data = [ ]
    self.token_order = ['seconds', 'lat', 'long', 'height',
        'v_north', 'v_east', 'v_up', 'rot_y', 'rot_x', 
        'azimuth', 'week'];
    f = open(filename);
    for line in f:
      l = line.rstrip()
      tokens = l.split(',');
      if len(tokens) == 21:
        record = { }
        for idx in range(10):
          record[self.token_order[idx]] = float(tokens[idx+10])
        # Parse out the gps week
        record[self.token_order[10]] = long((tokens[9].split(';'))[1])
        self.data.append(record)

  def getData(self):
    return self.data;

  def getNumericData(self):
    arr = np.zeros([len(self.data), 10], np.float64);
    for t in range(len(self.data)):
      for j in range(10):
        arr[t,j] = self.data[t][self.token_order[j]]

    arr[:, 1] = smoothData(arr[:, 1])
    arr[:, 2] = smoothData(arr[:, 2])
    arr[:, 3] = smoothData(arr[:, 3])
    #arr[:, 7] = smoothData(arr[:, 7])
    #arr[:, 8] = smoothData(arr[:, 8])
    #arr[:, 9] = smoothData(arr[:, 9])

    return arr;


