# usage:
# python test_video_reader.py <path to video splits>

import sys
from VideoReader import VideoReader

if __name__ == '__main__':
  reader = VideoReader(sys.argv[1])


