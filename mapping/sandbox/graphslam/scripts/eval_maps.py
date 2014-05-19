import h5py
import json
import numpy as np
from os.path import splitext, basename, join as pjoin
from pipeline_utils import print_and_call
from pipeline_config import MAPPING_PATH
from graphslam_config import GRAPHSLAM_EVAL_DIR, GRAPHSLAM_MAPS_DIR, MATCH_JSON_DATA
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt


'''
Evaluate alignment between maps
First compute bounds of map features to compare
Then compute score
'''

# TODO Parallelize

if __name__ == '__main__':
    # First compute bounding box / center data
    bounds_file_pairs = list()
    for match in MATCH_JSON_DATA:
        fstem1 = '--'.join(match['rss1'])
        fstem2 = '--'.join(match['rss2'])

        map_data1 = pjoin(GRAPHSLAM_MAPS_DIR, fstem1 + '.h5')
        map_data2 = pjoin(GRAPHSLAM_MAPS_DIR, fstem2 + '.h5')
        pcd1 = splitext(map_data1)[0] + '.pcd'
        pcd2 = splitext(map_data2)[0] + '.pcd'

        # First convert to pcd format
        cmd = '%s/bin/h5_to_pcd --h5 %s --pcd %s' % (MAPPING_PATH, map_data1, pcd1)
        print_and_call(cmd)
        cmd = '%s/bin/h5_to_pcd --h5 %s --pcd %s' % (MAPPING_PATH, map_data2, pcd2)
        print_and_call(cmd)

        # Then compute the bounding boxes
        bounds1 = '%s/%s.h5' % (GRAPHSLAM_EVAL_DIR, splitext(basename(pcd1))[0])
        bounds2 = '%s/%s.h5' % (GRAPHSLAM_EVAL_DIR, splitext(basename(pcd2))[0])
        cmd = '%s/bin/compute_bounds %s %s' % (MAPPING_PATH, pcd1, bounds1)
        print_and_call(cmd)
        cmd = '%s/bin/compute_bounds %s %s' % (MAPPING_PATH, pcd2, bounds2)
        print_and_call(cmd)

        bounds_file_pairs.append((bounds1, bounds2))

    # Now compute match quality
    eval_json = dict()
    eval_json['match_eval'] = list()
    for (file_pair, match_data) in zip(bounds_file_pairs, MATCH_JSON_DATA):
        h5f = h5py.File(file_pair[0], 'r')
        centers1 = h5f['cluster_centers'][...]
        h5f.close()
        h5f = h5py.File(file_pair[1], 'r')
        centers2 = h5f['cluster_centers'][...]
        h5f.close()

        # Compute nearest neighbor distances

        nbrs = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(centers1)
        distances, tgt_indices = nbrs.kneighbors(centers2)
        distances = distances.ravel()
        distances = distances[distances < 2.0]  # PARAM

        # Also compute lateral distances when not taking z coord into account

        centers1_lat = np.array(centers1)
        centers2_lat = np.array(centers2)
        centers1_lat[:, 2] = 0.0
        centers2_lat[:, 2] = 0.0
        nbrs_lat = NearestNeighbors(n_neighbors=1, algorithm='ball_tree').fit(centers1_lat)
        distances_lat, tgt_indices_lat = nbrs_lat.kneighbors(centers2_lat)
        distances_lat = distances_lat.ravel()
        distances_lat = distances_lat[distances_lat < 2.0]  # PARAM

        # Compute aggregate statistics

        mean_dist = np.mean(distances)
        median_dist = np.median(distances)

        mean_dist_lat = np.mean(distances_lat)
        median_dist_lat = np.median(distances_lat)

        # Store

        match_eval = dict()
        match_eval['match_data'] = match_data
        match_eval['match_distances'] = {'mean_dist': mean_dist, 'distances': distances.tolist(), 'median_dist': median_dist}
        match_eval['match_distances_lat'] = {'mean_dist': mean_dist_lat, 'distances': distances_lat.tolist(), 'median_dist': median_dist_lat}
        eval_json['match_eval'].append(match_eval)

        # Save evaluation summary

        eval_json_file = '%s/%s.json' % (GRAPHSLAM_EVAL_DIR, '+'.join((fstem1, fstem2)))
        json.dump(eval_json, open(eval_json_file, 'w'), indent=4, sort_keys=True)

        # Save figures

        plt.hist(distances.tolist(), 50)
        plt.ylabel('count')
        plt.xlabel('distance')
        fig_file = '%s/%s_hist.pdf' % (GRAPHSLAM_EVAL_DIR, '+'.join((fstem1, fstem2)))
        plt.savefig(fig_file)

        plt.hist(distances_lat.tolist(), 50)
        plt.ylabel('count')
        plt.xlabel('lateral distance')
        fig_file = '%s/%s_hist_lat.pdf' % (GRAPHSLAM_EVAL_DIR, '+'.join((fstem1, fstem2)))
        plt.savefig(fig_file)
