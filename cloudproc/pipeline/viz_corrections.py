import h5py
import glob
from os.path import join as pjoin
import numpy as np
from pipeline_config import ICP_TRANSFORMS_DIR
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from pipeline_config import EXPORT_START, EXPORT_NUM, EXPORT_STEP
import argparse


def read_imu_transforms(gps_file):
    gps_reader = GPSReader(gps_file)
    GPSData = gps_reader.getNumericData()
    imu_transforms = IMUTransforms(GPSData)
    return imu_transforms


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize ICP corrections')
    parser.add_argument('coord', type=int, help='coordinate to visualize', choices=range(0,3))
    args = parser.parse_args()

    # Transforms given by ICP

    h5_files = glob.glob(pjoin(ICP_TRANSFORMS_DIR, '*.h5'))
    num_frames = len(h5_files)

    coord = args.coord
    coord_name = ['x', 'y', 'z'][coord]

    delta_coord_icp = list()
    delta_rot_icp = list()

    for k in range(1, num_frames):
        f = pjoin(ICP_TRANSFORMS_DIR, '%d.h5' % k)
        print f
        h5f = h5py.File(f, 'r')
        transform_dset = h5f['transform']
        # TODO Also use fitness score? May need to normalize first
        transform_mat = transform_dset[...]
        R = transform_mat[0:3, 0:3]
        t = transform_mat[0:3, 3]
        delta_coord_icp.append(t[coord])

    # Transforms given by GPS

    imu_transforms = read_imu_transforms('/media/sdb/17N_b2/17N_b_gps.out')

    t = np.arange(EXPORT_START, EXPORT_START + (EXPORT_NUM-1)*EXPORT_STEP, EXPORT_STEP)
    print t
    print len(t)
    print len(delta_coord_icp)
    delta_z_gps = imu_transforms[t + EXPORT_STEP, coord, 3] - imu_transforms[t, coord, 3]

    # Plot and compare

    # Plot of change in coord

    import matplotlib.pyplot as plt
    corrected = np.array(delta_coord_icp) + np.array(delta_z_gps)
    p1 = plt.plot(range(1, num_frames), delta_z_gps, label='$\Delta %s_{\mathrm{gps}}$' % coord_name)
    p2 = plt.plot(range(1, num_frames), np.array(delta_coord_icp), label='$\delta %s_{\mathrm{icp}}$' % coord_name)
    p3 = plt.plot(range(1, num_frames), corrected, label='$\Delta %s_{\mathrm{gps}} + \delta %s_{\mathrm{icp}}$' % (coord_name, coord_name))

    handles, labels = plt.gca().get_legend_handles_labels()
    plt.legend(handles, labels, prop={'size': 20})

    plt.show()
