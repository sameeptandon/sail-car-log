from Q50_config import *
import sys, os
from GPSReader import *
from GPSReprojection import *
from GPSTransforms import *
import numpy as np

if __name__ == '__main__': 

    video_filename = sys.argv[1] 
    path, vfname = os.path.split(video_filename)
    vidname = vfname.split('.')[0]
    cam_num = int(vidname[-1])
    gps_filename = path + '/' + vidname[0:-1] + '_gps.out'
    gps_reader = GPSReader(gps_filename)
    GPSData = gps_reader.getNumericData() 

    # this has been flipped for the q50
    roll_start = deg2rad(GPSData[0,8])
    pitch_start = deg2rad(GPSData[0,7])
    yaw_start = -deg2rad(GPSData[0,9])
    base_R_to_i_from_w = R_to_i_from_w(roll_start, pitch_start, yaw_start) 

    pts = WGS84toENU(GPSData[0,1:4], GPSData[:,1:4])

    pts = np.dot(base_R_to_i_from_w, pts)
    print pts.transpose()
    



