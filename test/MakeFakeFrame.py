#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import sys
from random import randint

sys.path.append('../process')
sys.path.append('../lidar')
from GPSReader import GPSReader
from FrameFinder import utc_from_gps


def main():
    """ Makes a fake folder containing sample ldr file names. The folder will
    contain 1/5th the number of entries in the .out file to simulate a real
    capture.
    """

    if len(sys.argv) < 2:
        print """
        Usage: MakeFakeFrames <gps.out> [-p]
        -p    perturbs the times by -9 to 10 microseconds
        """
        sys.exit()

    gps_file_name = sys.argv[1]
    
    if len(sys.argv) == 3 and sys.argv[2] == "-p":
        perturb = True
    else:
        perturb = False
    
    reader = GPSReader(gps_file_name)

    camera_times = [utc_from_gps(data['week'], data['seconds'])
                    for data in reader.getData()]

    fake_dir_name = 'fake_frames/'
    os.mkdir(fake_dir_name)
    for (i, time) in enumerate(camera_times):
        if i % 5 == 0:
            if perturb == False:
                name = str(time)
            else:
                name = str(time + randint(-9, 10))
            # Touches the file
            open(fake_dir_name + name + '.ldr', 'w').close()


if __name__ == '__main__':
    main()
