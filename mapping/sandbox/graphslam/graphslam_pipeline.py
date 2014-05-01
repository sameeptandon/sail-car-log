from subprocess import check_call
from ruffus import files, follows, pipeline_run, pipeline_printout, pipeline_printout_graph
from graphslam_config import GPS_TGT, GPS_SRC, OPT_POS_FILE_TGT, OPT_POS_FILE_SRC,\
        GPS_MATCH_FILE, GRAPHSLAM_PATH, GPS_ALIGNMENT_FILE,\
        TGT_DIR, SRC_DIR
from pipeline_config import NUM_CPUS


# TODO Find matches between runs

@files(GPS_TGT, GPS_MATCH_FILE)
def match_traces(dummy, output_file):
    cmd = 'python %s/match_traces.py %s' % (GRAPHSLAM_PATH, GPS_MATCH_FILE)
    print cmd
    check_call(cmd, shell=True)


@follows('match_traces')
@files([(GPS_TGT, OPT_POS_FILE_TGT), (GPS_SRC, OPT_POS_FILE_SRC)])
def solve_qp(input_file, output_file):
    cmd = 'python %s/solve_qp.py %s %s %s' % (GRAPHSLAM_PATH,
            input_file, GPS_TGT, output_file)
    print cmd
    check_call(cmd, shell=True)


# TODO Enforce that this be run after solve_qp AND estimate_normals
# in pipeline.py for both src and tgt runs
@follows(match_traces)
@files(GPS_MATCH_FILE, GPS_ALIGNMENT_FILE)
def align_traces(input_file, output_file):
    # Align the first match of src to tgt
    cmd = 'python %s/align_runs.py %s %s %s %s' % (GRAPHSLAM_PATH, TGT_DIR, SRC_DIR, GPS_MATCH_FILE, output_file)
    print cmd
    check_call(cmd, shell=True)


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
        #'clean': clean  # TODO
    }

    for key in tasks:
        if key in CMDS:
            tasks[key]()
