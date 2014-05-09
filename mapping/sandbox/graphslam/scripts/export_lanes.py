import os
from os.path import join as pjoin
from pipeline_config import CAMERA
from pipeline_utils import print_and_call
from pipeline_config import NUM_CPUS
from graphslam_config import GRAPHSLAM_PATH, GRAPHSLAM_LANES_DIR, GRAPHSLAM_OPT_POS_DIR, GPS_FILES, RSS_LIST
from joblib import Parallel, delayed


def export_lane_data(gps_file, rss):
    # Read in matches file to see which lanes we want to generate
    route_dir = os.path.dirname(gps_file)
    route, segment, split = rss
    stem = '_'.join(os.path.basename(gps_file).split('_')[0:-1])
    video = '%s%d.avi' % (stem, CAMERA)
    fname = '--'.join((route, segment, split)) + '.npz'
    opt_pos = '%s/%s' % (GRAPHSLAM_OPT_POS_DIR, fname)
    cmd = 'python %s/lidar_integrator.py %s %s %s %s --export --full' % (GRAPHSLAM_PATH, route_dir, video, opt_pos, pjoin(GRAPHSLAM_LANES_DIR, fname))
    print_and_call(cmd)


if __name__ == '__main__':
    try:
        # Not sure why throws an error
        Parallel(n_jobs=NUM_CPUS-1)(delayed(export_lane_data)(gps_file, rss) for gps_file, rss in zip(GPS_FILES, RSS_LIST))
    except Exception as e:
        print e
        pass
