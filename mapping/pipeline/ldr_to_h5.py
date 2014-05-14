import math
import os
import argparse
import numpy as np
import h5py
from Q50_config import LoadParameters
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from LidarTransforms import loadLDR, loadLDRCamMap
from pipeline_config import EXPORT_STEP, EXPORT_START, EXPORT_NUM, LANE_FILTER,\
    PARAMS_TO_LOAD, OPT_POS_FILE, EXPORT_FULL_NUM_FILE

'''
Essentially just pieces from LidarIntegrator except avoids
storing the data for all time steps in memory

Writes full point clouds for scan matching later
'''

def interp_transforms(T1, T2, alpha):
    assert alpha <= 1
    T_avg = alpha * T1 + (1 - alpha) * T2
    ## Need to orthonormalize transform
    R = T_avg[0:3, 0:3]
    R = np.linalg.qr(R, mode='complete')[0]
    T_avg[0:3, 0:3] = R
    return T_avg


def transform_points_in_sweep(pts, times, fnum, imu_transforms):
    for time in set(times):
        mask = times == time

        # FIXME PARAM
        offset = (time / float(1e6)) / 0.02
        #assert offset <= 5.1, offset   # FIXME HACK
        offset = min(5, offset)

        ind1 = int(fnum - math.ceil(offset))
        ind2 = int(fnum - math.floor(offset))
        # FIXME Can't interpolate before 0 so first frame not shifted properly
        ind1, ind2 = max(ind1, 0), max(ind2, 0)
        transform = interp_transforms(imu_transforms[ind1, :, :], imu_transforms[ind2, :, :], offset / 5.0)

        # transform data into imu_0 frame
        pts[:, mask] = np.dot(transform, pts[:, mask])


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

    imu_transforms = np.load(OPT_POS_FILE)['data']
    # FIXME put this in solve_qp?
    N = imu_transforms.shape[0]

    # Assuming want to just export from start to end
    step = EXPORT_STEP
    if args.full:
        start = 0
        num_fn = GPSData.shape[0] / step
        open(EXPORT_FULL_NUM_FILE, 'w').write(str(num_fn))
    else:
        start = EXPORT_START
        num_fn = EXPORT_NUM
    end = start + num_fn * step

    trans_wrt_imu = imu_transforms[start:end, 0:3, 3]

    params = LoadParameters(PARAMS_TO_LOAD)
    ldr_map = loadLDRCamMap(args.map)

    for t in range(num_fn):
        fnum = start + t * step
        if (fnum - start) % (100 * step) == 0:  # PARAM
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

        # Microseconds till end of the sweep
        times = data[:, 5]
        transform_points_in_sweep(pts, times, fnum, imu_transforms)

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

        '''
        # Also write the transform at that time
        transform = imu_transforms[fnum, :, :]
        h5f = h5py.File(os.path.join(args.h5_dir, '%d.transform' % t), 'w')
        dset = h5f.create_dataset('transform', transform.shape, dtype='f')
        dset[...] = transform
        h5f.close()
        '''
