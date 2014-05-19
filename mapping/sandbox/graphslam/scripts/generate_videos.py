from os.path import join as pjoin
from pipeline_utils import dset_dir_from_rss, print_and_call
from pipeline_config import CAMERA, NUM_CPUS
from graphslam_config import MATCH_JSON_DATA, GRAPHSLAM_VIDEOS_DIR, GRAPHSLAM_MAPS_DIR,\
        GRAPHSLAM_EVAL_DIR
from joblib import Parallel, delayed

def disp(s):
    print s

if __name__ == '__main__':
    cmds = list()
    for match in MATCH_JSON_DATA:
        video_file = pjoin(dset_dir_from_rss(match['rss1']), '_'.join(match['rss1'][1:]) + '%d.avi' % CAMERA)
        fstem1 = '--'.join(match['rss1'])
        fstem2 = '--'.join(match['rss2'])
        outvideo = pjoin(GRAPHSLAM_VIDEOS_DIR, '+'.join((fstem1, fstem2)) + '.avi')
        align_data = pjoin(GRAPHSLAM_MAPS_DIR, '+'.join((fstem1, fstem2)) + '.npz')
        labels1 = pjoin(GRAPHSLAM_EVAL_DIR, fstem1 + '.h5')
        labels2 = pjoin(GRAPHSLAM_EVAL_DIR, fstem2 + '.h5')
        cmd = 'python project_map_on_video.py {vid} {align_data} {labels1} {labels2} {out} --cam {cam}'.format(
                vid=video_file, align_data=align_data, out=outvideo,
                labels1=labels1, labels2=labels2, cam=CAMERA)
        cmds.append(cmd)
    #print cmds
    Parallel(n_jobs=NUM_CPUS-1)(delayed(print_and_call)(cmd) for cmd in cmds)
