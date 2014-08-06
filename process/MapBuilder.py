# usage: nobody knows, don't ask

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
import cv2
import math


class MapBuilder(object):
    def __init__(self, args, start_time, end_time, step_time, scan_window):
        """
        start_time: seconds into the GPS Mark 1 log
        end_time: seconds into the GPS Mark 1 log
        step_time: seconds between adding points from scans
        scan_window: +/- seconds of data to add between each step
        """
        self.args = args
        self.params = self.args['params']

        self.gps_reader_mark1 = GPSReader(self.args['gps_mark1'])
        self.gps_data_mark1 = self.gps_reader_mark1.getNumericData()

        self.lidar_loader = LDRLoader(self.args['frames'])
        self.imu_transforms_mark1 = IMUTransforms(self.gps_data_mark1)
        self.gps_times_mark1 = utc_from_gps_log_all(self.gps_data_mark1)

        self.T_from_l_to_i = self.params['lidar']['T_from_l_to_i']
        self.T_from_i_to_l = np.linalg.inv(self.T_from_l_to_i)

        # grab the initial time off the gps log and compute start and end times
        self.start_time = self.gps_times_mark1[0] + start_time * 1e6
        self.end_time = self.gps_times_mark1[0] + end_time * 1e6
        if self.end_time > self.gps_times_mark1[-1]:
            self.end_time = self.gps_times_mark1[-1]
        self.step_time = step_time
        self.scan_window = scan_window

        self.all_data = []
        self.all_t = []

    def buildMap(self, filters=None):
        """
        Creates a map with a set of filters
        filters: A list of filters.
            'ground': Leaves the ground points
            'lanes': Filters out the lane markings
            'forward': Only takes the top half of the scan. This ensures points
                       appear in chronolgical order
            'flat': Makes all x distances 0. This makes lane clustering better
        """
        self.all_data = []
        self.all_t = []

        current_time = self.start_time
        print (self.end_time - current_time) / 1e6
        while current_time + self.scan_window < self.end_time:
            # load points w.r.t lidar at current time
            data, t_data = self.lidar_loader.loadLDRWindow(current_time,
                                                           self.scan_window)
            if data is None:
                current_time += self.step_time * 1e6
                continue

            if filters != None:
                if 'ground' in filters:
                    filter_mask = data[:, 3] < 10
                else:
                    filter_mask = data[:, 3] > 30
                if 'lanes' in filters:
                    filter_mask &=  (data[:,1] < 3) & (data [:,1] > -3) &\
                                    (data[:,2] < -1.9) & (data[:,2] > -2.1)
                if 'forward' in filters:
                    filter_mask &= (data[:, 0] > 0)
                if 'no-trees' in filters:
                    filter_mask &= (data[:,1] < 30) & (data [:,1] > -30) &\
                                   (data[:,2] < -1) & (data[:,2] > -3)
                if 'flat' in filters:
                    data[:, 0] = 0

            data = data[filter_mask, :]
            t_data = t_data[filter_mask]

            # put in imu_t frame
            pts = data[:, 0:3].transpose()
            pts = np.vstack((pts, np.ones((1, pts.shape[1]))))
            pts = np.dot(self.T_from_l_to_i, pts)

            # Shift points according to timestamps instead of using
            # transform of full sweep.
            # This will put pts in imu_0 frame
            transform_points_by_times(pts, t_data, self.imu_transforms_mark1,
                                      self.gps_times_mark1)

            data[:, 0:3] = pts[0:3, :].transpose()
            self.all_data.append(data.copy())
            self.all_t.append(t_data.copy())

            current_time += self.step_time * 1e6

        print (self.end_time - current_time) / 1e6



    def exportData(self, file_name):
        print 'exporting data'
        export_data = np.row_stack(self.all_data)
        export_t = np.concatenate(self.all_t)
        print export_data
        print export_data.shape
        print export_t.shape
        np.savez(file_name, data=export_data, t=export_t)
        print 'export complete'


if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    mb = MapBuilder(args, 1, 600, 0.5, 0.1)
    mb.buildMap(['no-trees'])
    mb.exportData(sys.argv[3])
