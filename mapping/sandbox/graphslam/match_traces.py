'''
Figure out for each route which other route it overlaps with the most
Then compute and store the NN-matches as well as a JSON file describing
the assignments
'''

from os.path import join as pjoin
import json
import h5py
import argparse
import numpy as np
from numpy.linalg import norm
from gps_viewer import read_gps_fields
from sklearn.neighbors import NearestNeighbors
from WGS84toENU import WGS84toECEF
import matplotlib.pyplot as plt
from graphslam_config import GPS_MATCH_DIST_TOL,\
    GPS_BBOX_OVERLAP_PADDING, MIN_OVERLAP_THRESH, FREEWAY, DATE_RANGE
import datetime
from gps_viewer import get_route_segment_split_gps


def nn_matches(xyz_tgt, xyz_src, gps_dist_tol, debug=False):
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(xyz1)
    distances, tgt_indices = nbrs.kneighbors(xyz2)
    tgt_indices = tgt_indices.ravel()
    distances = distances.ravel()
    src_indices = np.array(xrange(tgt_indices.shape[0]), dtype=np.int64)

    # Sanity check
    if (debug):
        plt.plot(distances)
        plt.plot(tgt_indices / 100.0)
        plt.show()

    src_indices = src_indices[distances < gps_dist_tol]
    tgt_indices = tgt_indices[distances < gps_dist_tol]
    return src_indices, tgt_indices
    #print src_indices.shape
    #print tgt_indices.shape


def get_bbox(xyz):
    xyz_min = np.amin(xyz, axis=0)
    xyz_max = np.amax(xyz, axis=0)
    bbox_bounds = [xyz_min, xyz_max]
    return bbox_bounds


def bbox_overlap(bbox1, bbox2, padding=GPS_BBOX_OVERLAP_PADDING):
    xyz_min1 = bbox1[0]
    xyz_min2 = bbox2[0]
    xyz_max1 = bbox1[1]
    xyz_max2 = bbox2[1]
    bbox_bounds = [np.maximum(xyz_min1, xyz_min2) - padding, np.minimum(xyz_max1, xyz_max2) + padding]
    overlap = np.maximum(np.array([0, 0, 0]), bbox_bounds[1] - bbox_bounds[0])
    overlap_vol = np.prod(overlap)
    overlap_frac = 2 * overlap_vol / ((np.prod(xyz_max1 - xyz_min1) + np.prod(xyz_max2 - xyz_min2)))
    return overlap, overlap_vol, overlap_frac


def trace_dir(xyz):
    d = xyz[-1, :] - xyz[0, :]
    return d / norm(d)


def rad2deg(rad):
    return rad * 180.0 / np.pi


def deg_between(d1, d2):
    return rad2deg(np.arccos(np.dot(d1, d2)))


def read_gps_ecef(gps_file):
    # Read and pre-process
    llh = read_gps_fields(gps_file, ['lat', 'long', 'height'])
    if len(llh[0]) == 0:
        return None
    llh = np.array(llh, dtype=np.float64).T
    # NOTE Working w/ ECEF
    xyz = WGS84toECEF(llh).T
    return xyz


def avg_vel(xyz):
    return sum(norm(xyz[1:, :] - xyz[0:-1, :], axis=1)) / xyz.shape[0] * 50


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Find matches between GPS traces')
    parser.add_argument('out_dir', help='directory to save the matches to')
    parser.add_argument('--debug', action='store_true', help='display plots and don\'t save')
    args = parser.parse_args()

    route_segment_split_gps = get_route_segment_split_gps()
    freeway_data = json.load(open('segment_freeways.json', 'r'))
    gps_file_list = list()
    gps_bbox_list = list()
    gps_trace_dirs = list()
    rss_list = list()

    for route in route_segment_split_gps.keys():
        month, day, year = [int(x) for x in route.split('-')[0:3]]
        year = 2000 + year
        if not (DATE_RANGE[0] <= datetime.date(year, month, day) <= DATE_RANGE[1]):
            continue
        if route not in freeway_data:
            # Unlabeled route
            continue
        print route
        segment_split_gps = route_segment_split_gps[route]
        for segment in segment_split_gps:
            print '\t' + segment
            split_gps = segment_split_gps[segment]
            for split in split_gps:
                if FREEWAY not in freeway_data[route][segment][split].split('+'):
                    # Only consider splits on freeway we're currently looking at
                    continue

                gps_file = split_gps[split]['gps_file']
                print '\t\t' + split + ': ' + gps_file
                xyz = read_gps_ecef(gps_file)
                if xyz is None:
                    continue
                bbox = get_bbox(xyz)
                gps_file_list.append(gps_file)
                gps_bbox_list.append(bbox)
                gps_trace_dirs.append(trace_dir(xyz))
                rss_list.append((route, segment, split))

    N = len(gps_bbox_list)
    overlap_fracs = np.zeros((N, N))
    for j in range(N):
        for k in range(N):
            overlap_fracs[j, k] = bbox_overlap(gps_bbox_list[j], gps_bbox_list[k], padding=0)[2]

    print np.max(overlap_fracs)

    if args.debug:
        plt.imshow(overlap_fracs, interpolation='nearest')
        plt.colorbar()
        plt.show()

    json_data = dict()
    json_data['matches'] = list()

    # TODO FIXME Could be excluding some matches here
    seen = np.zeros((N, N), dtype=np.bool8)

    for j in range(N):
        overlap_fracs[j, j] = -1
        k = overlap_fracs[j, :].argmax()
        if seen[j, k]:
            continue

        max_overlap = overlap_fracs[j, k]

        xyz1 = read_gps_ecef(gps_file_list[j])
        xyz2 = read_gps_ecef(gps_file_list[k])
        deg = deg_between(trace_dir(xyz1), trace_dir(xyz2))

        # We'll only look at paths that overlap by more than a threshold for now
        # and also go in the same direction # PARAM
        if max_overlap < MIN_OVERLAP_THRESH or deg > 90:
            continue

        # Not necessarily the same length
        print 'trace lengths:', xyz1.shape[0], xyz2.shape[0]
        print 'avg. vel. (m/s):', avg_vel(xyz1), ',', avg_vel(xyz2)
        print j, k, max_overlap, deg

        rss1, rss2 = rss_list[j], rss_list[k]
        match_file = pjoin(args.out_dir, '+'.join(['--'.join(rss1), '--'.join(rss2)]) + '.h5')

        match_json = {
            'rss1': rss1,
            'rss2': rss2,
            'gps_file1': gps_file_list[j],
            'gps_file2': gps_file_list[k],
            'match_file': match_file
        }

        # Compute the nearest neighbors and retain those within certain distance
        src_indices, tgt_indices = nn_matches(xyz1, xyz2, GPS_MATCH_DIST_TOL)
        if src_indices.shape[0] == 0 or tgt_indices.shape[0] == 0:
            continue

        json_data['matches'].append(match_json)
        if not args.debug:
            match_mat = np.hstack((src_indices.reshape((-1, 1)), tgt_indices.reshape((-1, 1))))
            print match_mat
            h5f = h5py.File(match_file, 'w')
            dset = h5f.create_dataset('matches', match_mat.shape, dtype='i')
            dset[...] = match_mat

            h5f.close()

        seen[j, k] = True
        seen[k, j] = True

    json_out_file = pjoin(args.out_dir, 'matches.json')
    json.dump(json_data, open(json_out_file, 'w'), indent=4, sort_keys=True)
