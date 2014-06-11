import numpy as np
from scipy.ndimage.filters import convolve1d

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

class GPSCovarianceReader():
    def __init__(self,filename):
        from math import sqrt
        f = open(filename);
        self.nextLineInsCov = False
        self.data = [ ]
        self.token_order = ['tx','ty','tz','rx','ry','rz']
        for line in f:
            if 'INSCOV' in line: 
                self.nextLineInsCov = True
            elif self.nextLineInsCov:
                record = { } 
                self.nextLineInsCov = False 
                tokens = line.split(' ')
                offset = 7
                record['tx'] = sqrt(float(tokens[offset + 0]))
                record['ty'] = sqrt(float(tokens[offset + 4]))
                record['tz'] = sqrt(float(tokens[offset + 8]))
                offset += 9
                record['rx'] = sqrt(float(tokens[offset + 0]))
                record['ry'] = sqrt(float(tokens[offset + 4]))
                record['rz'] = sqrt(float(tokens[offset + 8]))
                self.data.append(record)

    def getNumericData(self):
        arr = np.zeros([len(self.data), 6], np.float64);
        for t in range(len(self.data)):
            for j in range(6):
                arr[t,j] = self.data[t][self.token_order[j]]
        return arr

class GPSReader():
  def __init__(self, filename):
    self.data = [ ]
    self.token_order = ['seconds', 'lat', 'long', 'height',
        'v_north', 'v_east', 'v_up', 'rot_y', 'rot_x',
        'azimuth', 'week'];
    f = open(filename);
    for line in f:
      l = line.rstrip()
      tokens = l.split(',')
      for k in range(len(tokens)):
          if 'PVA' in tokens[k]:
              break
      tokens = tokens[k:]
      # for TAGGEDMARKxPVA logs, delete the tag, and then process as is
      #print tokens
      if len(tokens) == 22:
          del tokens[11] # eventID token
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
    arr = np.zeros([len(self.data), 11], np.float64);
    for t in range(len(self.data)):
      for j in range(11):
        arr[t,j] = self.data[t][self.token_order[j]]


    #arr[:, 1] = smoothData(arr[:, 1])
    #arr[:, 2] = smoothData(arr[:, 2])
    #arr[:, 3] = smoothData(arr[:, 3])
    #arr[:-1, 3] = smoothDerivativeData(arr[:, 3], wind=251)
    #arr[:-1, 1] = holdDerivativeData(arr[:, 1])
    #arr[:-1, 2] = holdDerivativeData(arr[:, 2])
    #arr[0:-1, 3] = holdDerivativeData(arr[:, 3], clipVal=0.1)
    #arr[:, 3] = smoothData(arr[:, 3])
    #arr[:, 7] = smoothData(arr[:, 7])
    #arr[:, 8] = smoothData(arr[:, 8])
    #arr[:, 9] = smoothData(arr[:, 9])

    return arr;


