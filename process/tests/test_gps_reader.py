# usage:
# python test_gps_reader.py <path to gps log>

import sys
from GPSReader import GPSReader

if __name__ == '__main__':
  reader = GPSReader(sys.argv[1])
  print reader.getNumericData();
  print reader.getNumericData()[:,1]

