#usage: nobody knows, don't ask

from Q50_config import *
from ArgParser import *
import sys
import os
from GPSReader import *
from GPSTransforms import *
from VideoReader import *
from LidarTransforms import *
from VtkRenderer import *
from transformations import euler_matrix
import numpy as np
from ColorMap import *
import vtk
import copy
import cv2
import math

START_TIME = 1.0  # seconds into the GPS Mark 1 log
END_TIME = 10.0  # seconds into the GPS Mark 1 log
STEP_TIME = 0.2  # seconds between adding points from scans
SCAN_WINDOW = 0.05  # +/- seconds of data to add between each step

def exportData(all_data, all_t):
        print 'exporting data'
        export_data = np.row_stack(all_data)
        export_t = np.concatenate(all_t)
        print export_data
        print export_data.shape
        print export_t.shape
        np.savez(sys.argv[3], data=export_data, t=export_t)
        print 'export complete'

if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    params = args['params']

    gps_reader_mark1 = GPSReader(args['gps_mark1'])
    gps_data_mark1 = gps_reader_mark1.getNumericData()

    lidar_loader = LDRLoader(args['frames'])
    imu_transforms_mark1 = IMUTransforms(gps_data_mark1)
    gps_times_mark1 = utc_from_gps_log_all(gps_data_mark1)

    T_from_l_to_i = params['lidar']['T_from_l_to_i']
    T_from_i_to_l = np.linalg.inv(T_from_l_to_i)

    # grab the initial time off the gps log and compute start and end times
    current_time = gps_times_mark1[0] + START_TIME * 1e6
    end_time = gps_times_mark1[0] + END_TIME * 1e6

    all_data = [ ]
    all_t = [ ]
    # in this interval, we will build a map
    while current_time < end_time:
        print (end_time - current_time) / 1e6
        # load points w.r.t lidar at current time
        data, t_data = lidar_loader.loadLDRWindow(
            current_time, SCAN_WINDOW)

        # TODO: filter points

        # put in imu_t frame
        pts = data[:, 0:3].transpose()
        pts = np.vstack((pts, np.ones((1, pts.shape[1]))))
        pts = np.dot(T_from_l_to_i, pts)

        # Shift points according to timestamps instead of using
        # transform of full sweep. 
        # This will put pts in imu_0 frame
        transform_points_by_times(pts, t_data, imu_transforms_mark1,
                                  gps_times_mark1)

        data[:, 0:3] = pts[0:3, :].transpose()
        all_data.append(data.copy())
        all_t.append(t_data.copy())

        current_time += STEP_TIME * 1e6

    exportData(all_data, all_t)
