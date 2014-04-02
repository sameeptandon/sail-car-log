from ruffus import follows, transform, regex, mkdir,\
        pipeline_printout, pipeline_printout_graph,\
        pipeline_run, files, split, merge, collate,\
        touch_file, posttask
import os
import sys
from subprocess import check_call
from pipeline_config import DATA_DIR, LDR_DIR, POINTS_H5_DIR,\
        PCD_DIR, PCD_DOWNSAMPLED_DIR, NUM_CPUS, DSET_DIR, DSET,\
        SAIL_CAR_LOG_PATH, CLOUDPROC_PATH, DOWNSAMPLE_LEAF_SIZE,\
        K_NORM_EST, PCD_DOWNSAMPLED_NORMALS_DIR


dirs = [DATA_DIR, LDR_DIR, POINTS_H5_DIR, PCD_DIR,
        PCD_DOWNSAMPLED_DIR, PCD_DOWNSAMPLED_NORMALS_DIR]
MKDIRS = [mkdir(d) for d in dirs]

# NOTE chdir into dset dir so can just specify relative paths to data
os.chdir(DSET_DIR)


@follows(*MKDIRS)
@posttask(touch_file('%s/sentinel' % POINTS_H5_DIR))
def convert_ldr_to_h5():
    if os.path.exists('%s/sentinel' % POINTS_H5_DIR):
        return
    LIDAR_INTEGRATOR = '%s/process/LidarIntegrator.py' % SAIL_CAR_LOG_PATH
    cmd = 'python {integrator} {dset_dir} {dset}.avi {h5_dir} --export --all --h5'.format(integrator=LIDAR_INTEGRATOR, dset_dir=DSET_DIR, h5_dir=POINTS_H5_DIR, dset=DSET)
    check_call(cmd, shell=True)


@follows('convert_ldr_to_h5')
@transform('./h5/*.h5',
           regex('./h5/(.*?).h5'),
           r'./pcd/\1.pcd')
def convert_h5_to_pcd(input_file, output_file):
    h5_to_pcd = '%s/bin/h5_to_pcd' % CLOUDPROC_PATH
    cmd = '%s --h5 %s --pcd %s' % (h5_to_pcd, input_file, output_file)
    print cmd
    check_call(cmd, shell=True)


@follows('convert_h5_to_pcd')
@transform('./pcd/*.pcd',
           regex('./pcd/(.*?).pcd'),
           r'./pcd_downsampled/\1.pcd')
def downsample_pcds(input_file, output_file):
    downsampler = '%s/bin/downsample_cloud' % CLOUDPROC_PATH
    cmd = '%s --src_pcd %s --out_pcd %s --leaf_size %f' % (downsampler, input_file,
                output_file, DOWNSAMPLE_LEAF_SIZE)
    print cmd
    check_call(cmd, shell=True)


@follows('downsample_pcds')
@transform('./pcd_downsampled/*.pcd',
           regex('./pcd_downsampled/(.*?).pcd'),
           r'./pcd_downsampled_normals/\1.pcd')
def estimate_normals(input_file, output_file):
    norm_est = '%s/bin/estimate_normals' % CLOUDPROC_PATH
    cmd = '%s --src_pcd %s --out_pcd %s --k %d' % (norm_est, input_file,
                output_file, K_NORM_EST)
    print cmd
    check_call(cmd, shell=True)


@follows('estimate_normals')
def register_clouds():
    pass


@follows('register_clouds')
def build_static_map():
    pass


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Usage: ./train_pipeline.py print,graph,run (task1,task2)'
        sys.exit(1)

    TORUN = [
        'convert_ldr_to_h5'
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
    }

    for key in tasks:
        if key in CMDS:
            tasks[key]()
