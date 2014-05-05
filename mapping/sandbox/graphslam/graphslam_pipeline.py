import os
from subprocess import check_call
from ruffus import files, follows, pipeline_run, pipeline_printout, pipeline_printout_graph, touch_file,\
        posttask, jobs_limit
from graphslam_config import GRAPHSLAM_PATH,\
        GRAPHSLAM_MATCH_DIR, GRAPHSLAM_OPT_POS_DIR, GRAPHSLAM_ALIGN_DIR,\
        MATCHES_FILE, GPS_FILES, RSS_LIST, GRAPHSLAM_OUT_DIR, GRAPHSLAM_DIRS
import pipeline_config
from pipeline_config import NUM_CPUS, SAIL_CAR_LOG_PATH


# TODO Find matches between runs

@files(None, MATCHES_FILE)
def match_traces(dummy, output_file):
    cmd = 'python %s/match_traces.py %s' % (GRAPHSLAM_PATH, GRAPHSLAM_MATCH_DIR)
    print cmd
    check_call(cmd, shell=True)


def reload_config():
    reload(pipeline_config)


@follows('match_traces', reload_config)
@files(zip(GPS_FILES, [os.path.join(GRAPHSLAM_OPT_POS_DIR, '--'.join(rss) + '.npz') for rss in RSS_LIST], GPS_FILES))
def solve_qps(gps_src_file, output_file, gps_tgt_file):
    cmd = 'python %s/solve_qp.py %s %s %s' % (GRAPHSLAM_PATH,
            gps_src_file, gps_tgt_file, output_file)
    print cmd
    check_call(cmd, shell=True)


@follows('solve_qps')
@jobs_limit(1)
@files(MATCHES_FILE, '%s/run_pipeline_sentinel' % GRAPHSLAM_OUT_DIR)
@posttask(touch_file('%s/run_pipeline_sentinel' % GRAPHSLAM_OUT_DIR))
def run_pipelines(dummy, sentinel):
    for route, segment, split in RSS_LIST:
        cmd = 'export SCL_ROUTE=%s; export SCL_SEGMENT=%s; export SCL_SPLIT=%s; python %s/mapping/pipeline/pipeline.py run estimate_normals' % (route, segment, split, SAIL_CAR_LOG_PATH)
        print cmd
        check_call(cmd, shell=True)


@follows('run_pipelines')
@files('%s/run_pipeline_sentinel' % GRAPHSLAM_OUT_DIR,
    '%s/chunk_and_align_sentinel' % GRAPHSLAM_ALIGN_DIR)
@posttask(touch_file('%s/chunk_and_align_sentinel' % GRAPHSLAM_ALIGN_DIR))
def chunk_and_align(dummy, sentinel):
    cmd = 'python %s/chunk_and_align.py' % GRAPHSLAM_PATH
    print cmd
    check_call(cmd, shell=True)


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
