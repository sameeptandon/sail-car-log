import numpy as np

class GPSReader():
  def __init__(self, filename):
    self.data = [ ]
    self.token_order = ['seconds', 'lat', 'long', 'height',
        'v_north', 'v_east', 'v_up', 'rot_y', 'rot_x', 
        'azimuth'];
    f = open(filename);
    for line in f:
      l = line.rstrip()
      tokens = l.split(',');
      if len(tokens) == 21:
        record = { }
        for idx in range(10):
          record[self.token_order[idx]] = float(tokens[idx+10])
        self.data.append(record)

  def getData(self):
    return self.data;

  def getNumericData(self):
    arr = np.zeros([len(self.data), 10]);
    for t in range(len(self.data)):
      for j in range(10):
        arr[t,j] = self.data[t][self.token_order[j]]
    return arr;

