import os
from os.path import splitext, basename, join as pjoin
from pipeline_utils import print_and_call
from pipeline_config import MAPPING_PATH
from graphslam_config import GRAPHSLAM_EVAL_DIR, GRAPHSLAM_MAPS_DIR, MATCH_JSON_DATA


'''
Evaluate alignment between maps
First compute bounds of map features to compare
Then compute score
'''

# TODO Parallelize

if __name__ == '__main__':
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
