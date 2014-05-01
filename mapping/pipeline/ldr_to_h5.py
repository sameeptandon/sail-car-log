import os
import argparse
import numpy as np
import h5py
from Q50_config import LoadParameters
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms, R_to_i_from_w
from WGS84toENU import deg2rad
from LidarTransforms import loadLDR, loadLDRCamMap
from pipeline_config import EXPORT_STEP, EXPORT_START, EXPORT_NUM, LANE_FILTER,\
    PARAMS_TO_LOAD, OPT_POS_FILE

'''
Essentially just pieces from LidarIntegrator except avoids
storing the data for all time steps in memory

Writes full point clouds for scan matching later
'''


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Convert ldr files to h5 files containing points')
    parser.add_argument('gps', help='gps file')
    parser.add_argument('map', help='map file')
    parser.add_argument('h5_dir', help='directory to write h5 files to')
    parser.add_argument('--full', action='store_true')
    args = parser.parse_args()

    gps_reader = GPSReader(args.gps)
    GPSData = gps_reader.getNumericData()
    #imu_transforms = IMUTransforms(GPSData)
    opt_pos = np.load(OPT_POS_FILE)['data']
    # FIXME put this in solve_qp?
    N = opt_pos.shape[1]
    imu_transforms = np.zeros((N, 4, 4))
    for t in range(N):
        imu_transforms[t, :, :] = np.eye(4)
        imu_transforms[t, 0:3, 3] = opt_pos[:, t]
        roll = deg2rad(GPSData[t, 8])
        pitch = deg2rad(GPSData[t, 7])
        yaw = -deg2rad(GPSData[t, 9])
        R = R_to_i_from_w(roll, pitch, yaw)
        imu_transforms[t, 0:3, 0:3] = R.transpose()

    # Assuming want to just export from start to end
    step = EXPORT_STEP
    if args.full:
        start = 0
        num_fn = GPSData.shape[0] / step
    else:
        start = EXPORT_START
        num_fn = EXPORT_NUM
    end = start + num_fn * step

    trans_wrt_imu = imu_transforms[start:end, 0:3, 3]

    params = LoadParameters(PARAMS_TO_LOAD)
    ldr_map = loadLDRCamMap(args.map)

    for t in range(num_fn):
        fnum = start + t * step
        print '%d / %d' % (fnum, end)

        data = loadLDR(ldr_map[fnum])
        if data.shape[0] == 0:
            print '%d data empty' % t
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
                               (data[:, 0] > 0) & \
                               (data[:, 2] > -5)

        filtered_data = data[data_filter_mask, :]

        if filtered_data.shape[0] == 0:
            print '%d data empty after filtering' % t
            # FIXME, hack, just include a single point
            data = data[0:1, :]
            #raise
        else:
            data = filtered_data

        # transform data into IMU frame at time t
        pts = data[:, 0:3].transpose()
        pts = np.vstack((pts, np.ones((1, pts.shape[1]))))
        T_from_l_to_i = params['lidar']['T_from_l_to_i']
        pts = np.dot(T_from_l_to_i, pts)
        # transform data into imu_0 frame
        pts = np.dot(imu_transforms[fnum, :, :], pts)
        pts = pts.transpose()

        # for exporting purposes
        pts_copy = np.array(pts[:, 0:3])
        pts_copy = np.column_stack((pts_copy, np.array(data[:, 3])))
        pts_copy = np.column_stack((pts_copy, fnum*np.ones((pts.shape[0], 1))))

        fname = os.path.join(args.h5_dir, '%d.h5' % t)
        h5f = h5py.File(fname, 'w')
        dset = h5f.create_dataset('points', pts_copy.shape, dtype='f')
        dset[...] = pts_copy
        h5f.close()

        # Also write the transform at that time
        transform = imu_transforms[fnum, :, :]
        h5f = h5py.File(os.path.join(args.h5_dir, '%d.transform' % t), 'w')
        dset = h5f.create_dataset('transform', transform.shape, dtype='f')
        dset[...] = transform
        h5f.close()