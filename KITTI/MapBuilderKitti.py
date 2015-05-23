# usage: nobody knows, don't ask
import sys
sys.path.append('../process/')
from Q50_config import *
from ArgParser import *
import os
from GPSReaderKitti import *
from GPSTransforms import *
from LidarTransformsKitti import *
from transformations import euler_matrix
import numpy as np
from ColorMap import *
import cv2
import math


class MapBuilder(object):
    def __init__(self, args, start_time, end_time, step_time, scan_window, absolute=True):
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
        if absolute:
            self.imu_transforms_mark1 = absoluteTransforms(self.gps_data_mark1)
        else:
            self.imu_transforms_mark1 = IMUTransforms(self.gps_data_mark1)

        self.gps_times_mark1 = self.gps_data_mark1[:,0]*1e6#utc_from_gps_log_all(self.gps_data_mark1)

        self.T_from_l_to_i = self.params['lidar']['T_from_l_to_i']
        self.T_from_i_to_l = self.params['lidar']['T_from_i_to_l']

        # grab the initial time off the gps log and compute start and end times
        self.start_time = self.gps_times_mark1[0] + start_time * 1e6
        self.current_time = self.start_time
        self.end_time = self.gps_times_mark1[0] + end_time * 1e6
        if self.end_time > self.gps_times_mark1[-1]:
            self.end_time = self.gps_times_mark1[-1]
        self.step_time = step_time
        self.scan_window = scan_window

        self.all_data = []
        self.all_t = []

    def getFilterMask(self,data,filters):
        if 'ground' in filters:
            filter_mask = data[:, 3] < 10
        else:
            filter_mask = data[:, 3] > -300 #30
        if 'lanes' in filters:
            filter_mask &=  (data[:,1] < 10) & (data [:,1] > -10) &\
                            (data[:,2] < -1.95) & (data[:,2] > -2.05) &\
                            (data[:, 3] > 0.3)
        if 'road' in filters: # pts on the ground are assumed stationary
            filter_mask &=  (data[:,1] < 30) & (data [:,1] > -30) &\
                            (data[:,2] < -1.95) & (data[:,2] > -2.05)
        if 'forward' in filters:
            filter_mask &= (data[:, 0] > 0)
        if 'no-trees' in filters:
            filter_mask &= (data[:,1] < 30) & (data [:,1] > -30) &\
                           (data[:,2] < -1) & (data[:,2] > -3)
        if 'no-cars' in filters:  # pts on the ground or high up are assumed stationary
            filter_mask &= (data[:,1] < 20) & (data [:,1] > -20) &\
                           (((data[:,2] < -1.95) & (data[:,2] > -2.05) & (data[:, 3] > 30)) |\
                           (data[:,2] > 1.5))                  
        if 'above' in filters:  # pts high up are assumed stationary
            filter_mask &= ((data[:,1] > 2) | (data [:,1] < -2)) & (data[:,2] > -self.params['lidar']['height']+0.5)
        if 'flat' in filters:
            data[:, 0] = 0
        return filter_mask

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
        print (self.end_time - self.current_time) / 1e6
        while self.current_time + self.scan_window < self.end_time:
            data,t_data = self.getCurrentData(filters)
            if data is None or t_data is None:
              continue
            self.all_data.append(data.copy())
            self.all_t.append(t_data.copy())
            self.stepForward()
            
        print (self.end_time - self.current_time) / 1e6


    def getCurrentData(self, filters=None, local=False):
        # load points w.r.t lidar at current time
        data, t_data = self.lidar_loader.loadLDRWindow(self.current_time,
                                                       self.scan_window)
        print (self.current_time-self.gps_times_mark1[0])* 1e-6
        if data is None or data.shape[0] == 0:
            self.current_time += self.step_time * 1e6
            return None,None
        if filters != None:
            filter_mask = self.getFilterMask(data,filters)
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
                                  self.gps_times_mark1,local)
        data[:, 0:3] = pts[0:3, :].transpose()
        return data,t_data


    def stepForward(self):
      self.current_time += self.step_time * 1e6



    def getData(self):
        export_data = np.row_stack(self.all_data)
        export_t = np.concatenate(self.all_t)
        return (export_data, export_t)

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
    mb = MapBuilder(args, 1, 600, 0.1, 0.1,absolute=True)
    mb.buildMap(['no-cars'])
    mb.exportData(sys.argv[3])
