from os.path import join as pjoin
from pipeline_utils import print_and_call
from pipeline_config import NUM_CPUS
from graphslam_config import MATCH_JSON_DATA, GRAPHSLAM_OPT_POS_DIR, GRAPHSLAM_MAPS_DIR, GRAPHSLAM_ALIGN_DIR
from joblib import Parallel, delayed


if __name__ == '__main__':
    cmds = list()
    for match in MATCH_JSON_DATA:
        fstem1 = '--'.join(match['rss1'])
        fstem2 = '--'.join(match['rss2'])
        fname1 = fstem1 + '.npz'
        fname2 = fstem2 + '.npz'
        tb = pjoin(GRAPHSLAM_ALIGN_DIR, '+'.join((fstem1, fstem2)) + '.h5')
        align_data = pjoin(GRAPHSLAM_MAPS_DIR, '+'.join((fstem1, fstem2)) + '.npz')
        map_data1 = pjoin(GRAPHSLAM_MAPS_DIR, fstem1 + '.h5')
        map_data2 = pjoin(GRAPHSLAM_MAPS_DIR, fstem2 + '.h5')
        cmd = 'python align_maps.py {ldr1} {ldr2} {gps1} {gps2} {match} {t1} {t2} {tb} {align_data} {map_data1} {map_data2}'.format(
                ldr1=pjoin(GRAPHSLAM_MAPS_DIR, fname1), ldr2=pjoin(GRAPHSLAM_MAPS_DIR, fname2),
                gps1=match['gps_file1'], gps2=match['gps_file2'], match=match['match_file'],
                t1=pjoin(GRAPHSLAM_OPT_POS_DIR, fname1), t2=pjoin(GRAPHSLAM_OPT_POS_DIR, fname2),
                tb=tb, align_data=align_data, map_data1=map_data1, map_data2=map_data2)
        cmds.append(cmd)
    print cmds
    Parallel(n_jobs=NUM_CPUS-1)(delayed(print_and_call)(cmd) for cmd in cmds)
