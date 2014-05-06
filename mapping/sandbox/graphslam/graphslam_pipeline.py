import os
from os.path import join as pjoin
from subprocess import check_call
from joblib import Parallel, delayed
from ruffus import files, follows, pipeline_run, pipeline_printout, pipeline_printout_graph, jobs_limit
from graphslam_config import GRAPHSLAM_PATH,\
        GRAPHSLAM_MATCH_DIR, GRAPHSLAM_OPT_POS_DIR, GRAPHSLAM_ALIGN_DIR,\
        MATCHES_FILE, GPS_FILES, RSS_LIST, GRAPHSLAM_OUT_DIR, GRAPHSLAM_DIRS,\
        GRAPHSLAM_LANES_DIR, GRAPHSLAM_VIDEOS_DIR
import pipeline_config
from pipeline_config import NUM_CPUS, SAIL_CAR_LOG_PATH, CAMERA
from pipeline_utils import print_and_call, touchf


# TODO Find matches between runs

@files(None, MATCHES_FILE)
def match_traces(dummy, output_file):
    cmd = 'python %s/match_traces.py %s' % (GRAPHSLAM_PATH, GRAPHSLAM_MATCH_DIR)
    print_and_call(cmd)


def reload_config():
    reload(pipeline_config)


@follows('match_traces', reload_config)
@files(zip(GPS_FILES, [pjoin(GRAPHSLAM_OPT_POS_DIR, '--'.join(rss) + '.npz') for rss in RSS_LIST], GPS_FILES))
def solve_qps(gps_src_file, output_file, gps_tgt_file):
    cmd = 'python %s/solve_qp.py %s %s %s' % (GRAPHSLAM_PATH,
            gps_src_file, gps_tgt_file, output_file)
    print_and_call(cmd)


@follows('solve_qps')
@jobs_limit(1)
@files(MATCHES_FILE, '%s/run_pipeline_sentinel' % GRAPHSLAM_OUT_DIR)
def run_pipelines(dummy, sentinel):
    for route, segment, split in RSS_LIST:
        cmd = 'export SCL_ROUTE=%s; export SCL_SEGMENT=%s; export SCL_SPLIT=%s; python %s/mapping/pipeline/pipeline.py run estimate_normals' % (route, segment, split, SAIL_CAR_LOG_PATH)
        print_and_call(cmd)
    touchf('%s/run_pipeline_sentinel' % GRAPHSLAM_OUT_DIR)


@follows('run_pipelines')
@files('%s/run_pipeline_sentinel' % GRAPHSLAM_OUT_DIR, '%s/chunk_and_align_sentinel' % GRAPHSLAM_ALIGN_DIR)
def chunk_and_align(dummy, sentinel):
    cmd = 'python %s/chunk_and_align.py' % GRAPHSLAM_PATH
    print_and_call(cmd)
    touchf('%s/chunk_and_align_sentinel' % GRAPHSLAM_ALIGN_DIR)


@follows('chunk_and_align')
@files('%s/chunk_and_align_sentinel' % GRAPHSLAM_ALIGN_DIR,
    '%s/export_lanes_sentinel' % GRAPHSLAM_LANES_DIR)
def export_lanes(dummy, sentinel):
    cmd = 'python scripts/export_lanes.py'
    try:
        print_and_call(cmd)
    except Exception as e:
        print e
        pass
    touchf('%s/export_lanes_sentinel' % GRAPHSLAM_LANES_DIR)


@follows('export_lanes')
@files('%s/export_lanes_sentinel' % GRAPHSLAM_LANES_DIR,
    '%s/generate_videos_sentinel' % GRAPHSLAM_VIDEOS_DIR)
def generate_videos(dummy, sentinel):
    cmd = 'python scripts/generate_videos.py'
    print_and_call(cmd)
    touchf('%s/generate_videos_sentinel' % GRAPHSLAM_VIDEOS_DIR)


def clean():
    for d in GRAPHSLAM_DIRS:
        print 'deleting %s' % d
        if os.path.exists(d):
            check_call('rm -r %s' % d, shell=True)


if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print 'Usage: python graphslam_pipeline.py print,graph,run (task1,task2)'
        sys.exit(1)

    TORUN = [
    ]

    if len(sys.argv) == 3:
        TORUN = sys.argv[2].split(',')
    CMDS = sys.argv[1].split(',')

    tasks = {
        'print': lambda: pipeline_printout(sys.stdout, TORUN,
                                           forcedtorun_tasks=[], verbose=5),
        'graph': lambda: pipeline_printout_graph('graph.jpg', 'jpg', TORUN,
                                                 forcedtorun_tasks=[],
                                                 no_key_legend=False),
        'run': lambda: pipeline_run(TORUN,
                                    multiprocess=NUM_CPUS,
                                    one_second_per_job=False),
        'force': lambda: pipeline_run([],
                                      forcedtorun_tasks=TORUN,
                                      multiprocess=NUM_CPUS,
                                      one_second_per_job=False),
        'printf': lambda: pipeline_printout(sys.stdout,
                                            [],
                                            forcedtorun_tasks=TORUN,
                                            verbose=2),
        'clean': clean
    }

    for key in tasks:
        if key in CMDS:
            tasks[key]()
