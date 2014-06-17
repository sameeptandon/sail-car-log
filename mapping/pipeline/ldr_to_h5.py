import os
import argparse
import numpy as np
import h5py
from Q50_config import LoadParameters
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from LidarTransforms import loadLDR
from pipeline_config import LANE_FILTER, PARAMS_TO_LOAD, OPT_POS_FILE
from LidarIntegrator import transform_points_in_sweep

'''
Essentially just pieces from LidarIntegrator except avoids
storing the data for all time steps in memory

Writes full point clouds for scan matching later
'''

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert ldr files to h5 files containing points')
    parser.add_argument('gps_file')
    parser.add_argument('ldr_file')
    parser.add_argument('h5_file')
    parser.add_argument('--no_transform', action='store_true')
    args = parser.parse_args()

    if not args.no_transform:
        gps_reader = GPSReader(args.gps_file)
        gps_data = gps_reader.getNumericData()
        #imu_transforms = IMUTransforms(gps_data)
        imu_transforms = np.load(OPT_POS_FILE)['data']

    params = LoadParameters(PARAMS_TO_LOAD)

    # FIXME Assuming that ldr file named after frame num
    fnum = int(os.path.splitext(os.path.basename(args.ldr_file))[0])

    data = loadLDR(args.ldr_file)
    if data.shape[0] == 0:
        print '%d data empty' % fnum
        raise

    # Filter
    dist = np.sqrt(np.sum(data[:, 0:3] ** 2, axis=1))
    if LANE_FILTER:
        data_filter_mask = (dist > 3) & \
                           (data[:, 3] > 40) & \
                           (np.abs(data[:, 1]) < 2.2) & \
                           (np.abs(data[:, 1]) > 1.2) & \
                           (data[:, 2] < -1.8) & \
                           (data[:, 2] > -2.5)
    else:
        data_filter_mask = (dist > 3) & \
                           (data[:, 2] > -5)

    filtered_data = data[data_filter_mask, :]

    if filtered_data.shape[0] == 0:
        print '%d data empty after filtering' % fnum
        # FIXME, hack, just include a single point
        data = data[0:1, :]
        #raise
    else:
        data = filtered_data

    pts = data[:, 0:3].transpose()

    if not args.no_transform:
        # Transform data into IMU frame at time t
        pts = np.vstack((pts, np.ones((1, pts.shape[1]))))
        T_from_l_to_i = params['lidar']['T_from_l_to_i']
        pts = np.dot(T_from_l_to_i, pts)

        # Microseconds till end of the sweep
        # TODO Switch everything to use transform_points_by_times
        times = data[:, 5]
        transform_points_in_sweep(pts, times, fnum, imu_transforms)

    pts = pts.transpose()

    # for exporting purposes
    pts_copy = np.array(pts[:, 0:3])
    pts_copy = np.column_stack((pts_copy, np.array(data[:, 3])))
    pts_copy = np.column_stack((pts_copy, fnum*np.ones((pts.shape[0], 1))))

    h5f = h5py.File(args.h5_file, 'w')
    dset = h5f.create_dataset('points', pts_copy.shape, dtype='f')
    dset[...] = pts_copy
    h5f.close()
