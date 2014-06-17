import os
from os.path import join as pjoin
from pipeline_utils import print_and_call
from pipeline_config import NUM_CPUS, CAMERA, DATA_DIR
from graphslam_config import GRAPHSLAM_PATH, GRAPHSLAM_MAPS_DIR, GRAPHSLAM_OPT_POS_DIR, GPS_FILES, RSS_LIST
from joblib import Parallel, delayed


def export_map_data(gps_file, rss):
    route, segment, split = rss
    dset = '%s_%s' % (segment, split)
    dset_dir = pjoin(pjoin(DATA_DIR, route), dset)
    stem = '_'.join(os.path.basename(gps_file).split('_')[0:-1])
    video = '%s%d.avi' % (stem, CAMERA)
    fname = '--'.join((route, segment, split)) + '.npz'
    opt_pos = '%s/%s' % (GRAPHSLAM_OPT_POS_DIR, fname)
    cmd = 'python %s/lidar_integrator.py %s %s %s %s --export --full' % (GRAPHSLAM_PATH, dset_dir, video, opt_pos, pjoin(GRAPHSLAM_MAPS_DIR, fname))
    print_and_call(cmd)


if __name__ == '__main__':
    '''
    try:
        # Not sure why throws an error
        Parallel(n_jobs=NUM_CPUS-1)(delayed(export_lane_data)(gps_file, rss) for gps_file, rss in zip(GPS_FILES, RSS_LIST))
    except Exception as e:
        print e
        pass
    '''
    Parallel(n_jobs=3)(delayed(export_map_data)(gps_file, rss) for gps_file, rss in zip(GPS_FILES, RSS_LIST))
