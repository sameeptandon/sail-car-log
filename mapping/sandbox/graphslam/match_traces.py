'''
Figure out which segments of GPS traces overlap
'''

import h5py
import numpy as np
import argparse
from numpy.linalg import norm
from gps_viewer import read_gps_fields
from sklearn.neighbors import NearestNeighbors
from WGS84toENU import WGS84toENU, WGS84toECEF
import matplotlib.pyplot as plt
from graphslam_config import GPS_MATCH_DIST_TOL, GPS_TGT, GPS_SRC


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Find matches between GPS traces')
    parser.add_argument('out_file', help='file to save the matches to')
    parser.add_argument('--debug', action='store_true', help='display plots or not')
    args = parser.parse_args()

    llh1 = read_gps_fields(GPS_TGT, ['lat', 'long', 'height'])
    llh2 = read_gps_fields(GPS_SRC, ['lat', 'long', 'height'])

    llh1 = np.array(llh1, dtype=np.float64).T
    llh2 = np.array(llh2, dtype=np.float64).T

    xyz1 = WGS84toECEF(llh1).T
    xyz2 = WGS84toECEF(llh2).T
    #xyz1 = WGS84toENU(llh1[0, :], llh1).T
    #xyz2 = WGS84toENU(llh1[0, :], llh2).T

    # Compute the nearest neighbors and retain those within certain distance

    # Not necessarily the same length
    print 'trace lengths:'
    print llh1.shape[0], llh2.shape[0]

    print 'avg. vel. (m/s):'
    print sum(norm(xyz1[1:, :] - xyz1[0:-1, :], axis=1)) / xyz1.shape[0] * 50, ',',\
        sum(norm(xyz2[1:, :] - xyz2[0:-1, :], axis=1)) / xyz2.shape[0] * 50

    nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(xyz1)
    distances, tgt_indices = nbrs.kneighbors(xyz2)
    tgt_indices = tgt_indices.ravel()
    distances = distances.ravel()
    src_indices = np.array(xrange(tgt_indices.shape[0]), dtype=np.int64)

    # Sanity check
    if (args.debug):
        plt.plot(distances)
        plt.plot(tgt_indices / 100.0)
        plt.show()

    src_indices = src_indices[distances < GPS_MATCH_DIST_TOL]
    tgt_indices = tgt_indices[distances < GPS_MATCH_DIST_TOL]
    print src_indices.shape
    print tgt_indices.shape

    match_mat = np.hstack((src_indices.reshape((-1, 1)), tgt_indices.reshape((-1, 1))))
    print match_mat
    h5f = h5py.File(args.out_file, 'w')
    dset = h5f.create_dataset('matches', match_mat.shape, dtype='i')
    dset[...] = match_mat
    h5f.close()
