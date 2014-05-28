# usage:
# python lidar_integrator.py <dir> <basename><camnum>.avi <export name>.npz <optional additional flags such as --full>

# to change the type of data exported, see the function integrateClouds

import os
import sys
from ArgParser import parse_args
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from LidarTransforms import loadLDR, loadLDRCamMap
import numpy as np
import h5py
# TODO should be passed in as arguments
from pipeline_config import EXPORT_START, EXPORT_NUM, EXPORT_STEP
from LidarIntegrator import transform_points_in_sweep


all_data = list()

start_fn = EXPORT_START  # offset in frame numbers to start exporting data
num_fn = EXPORT_NUM  # number of frames to export. this is changed if --full is enabled
step = EXPORT_STEP  # step between frames

def exportData(data, fname, h5=False):
    print 'exporting data'
    #print data
    print data.shape
    if h5:
        f = h5py.File(fname, 'w')
        dset = f.create_dataset('points', data.shape, dtype='f')
        dset[...] = data
        f.close()
    else:
        np.savez(fname, data=data)
    print 'export complete'


def integrateClouds(ldr_map, imu_transforms, offset, num_steps, step, calibrationParameters):
    for t in range(num_steps):
        fnum = offset+t*step
        print fnum

        data = loadLDR(ldr_map[fnum])
        # filter out the roof rack
        dist = np.sqrt(np.sum(data[:, 0:3] ** 2, axis=1))

        # See LidarIntegrator.py to see how this is filtering
        data_filter_mask = (dist > 3)                   &\
                           (data[:, 3] > 40)            &\
                           (np.abs(data[:, 1]) < 2.2)   &\
                           (np.abs(data[:, 1]) > 1.2)   &\
                           (data[:, 2] < -1.8)          &\
                           (data[:, 2] > -2.5)

        data = data[data_filter_mask, :]

        # transform data into IMU frame at time t
        pts = data[:, 0:3].transpose()
        pts = np.vstack((pts, np.ones((1, pts.shape[1]))))
        T_from_l_to_i = calibrationParameters['lidar']['T_from_l_to_i']
        pts = np.dot(T_from_l_to_i, pts)

        # transform data into imu_0 frame

        # Microseconds till end of the sweep
        times = data[:, 5]
        transform_points_in_sweep(pts, times, fnum, imu_transforms)

        pts = pts.transpose()

        # for exporting purposes
        pts_copy = np.array(pts[:, 0:3])
        pts_copy = np.column_stack((pts_copy, np.array(data[:, 3])))
        pts_copy = np.column_stack((pts_copy, fnum*np.ones((pts.shape[0], 1))))
        all_data.append(pts_copy)


if __name__ == '__main__':
    vfname = sys.argv[2]
    vidname = vfname.split('.')[0]
    vidname2 = vidname[:-1] + '2'
    video_filename2 = sys.argv[1] + '/' + vidname2 + '.avi'

    args = parse_args(sys.argv[1], sys.argv[2])
    opt_pos_file = sys.argv[3]
    export_path = sys.argv[4]

    params = args['params']
    cam1 = params['cam'][0]
    cam2 = params['cam'][1]
    #gps_reader = GPSReader(args['gps'])
    #GPSData = gps_reader.getNumericData()
    #imu_transforms = IMUTransforms(GPSData)
    imu_transforms = np.load(opt_pos_file)['data']

    ldr_map = loadLDRCamMap(args['map'])


    if '--full' in sys.argv:
        total_num_frames = imu_transforms.shape[0]
        start_fn = 0
        step = 5
        num_fn = int(total_num_frames / step)

    integrateClouds(ldr_map, imu_transforms, start_fn, num_fn, step, params)

    data = np.row_stack(all_data)
    exportData(data, export_path, h5=False)
