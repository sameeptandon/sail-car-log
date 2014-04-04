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
        K_NORM_EST, PCD_DOWNSAMPLED_NORMALS_DIR, ICP_TRANSFORMS_DIR,\
        ICP_ITERS, ICP_MAX_DIST, REMOTE_DATA_DIR, REMOTE_FILES
from pipeline_utils import file_num

# TODO Commands to scp stuff over

dirs = [POINTS_H5_DIR, PCD_DIR, PCD_DOWNSAMPLED_DIR,
        PCD_DOWNSAMPLED_NORMALS_DIR, ICP_TRANSFORMS_DIR]
MKDIRS = [mkdir(d) for d in dirs]

# NOTE chdir into dset dir so can just specify relative paths to data
os.chdir(DSET_DIR)

DOWNLOADS = list()
for f in REMOTE_FILES:
    DOWNLOADS.append([None, f])

@follows(*MKDIRS)
@files(DOWNLOADS)
def download_files(dummy, local_file):
    cmd = 'rsync -vr --ignore-existing %s/%s .' % (REMOTE_DATA_DIR, local_file)
    print cmd
    check_call(cmd, shell=True)


@follows("download_files")
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
@transform('./pcd_downsampled_normals/*.pcd',
           regex('./pcd_downsampled_normals/(.*?).pcd'),
           r'./icp_transforms/\1.h5')
def align_clouds(input_file, output_file):
    icp_reg = '%s/bin/align_clouds' % CLOUDPROC_PATH
    if file_num(input_file) == 0:  # no transform for first pcd, touch empty file
        check_call('touch %s' % output_file, shell=True)
        return
    # cloud to apply transform to
    src = input_file
    # cloud to align to (previous index)
    tgt = os.path.join(os.path.dirname(input_file), str(file_num(input_file) - 1) + '.pcd')
    cmd = '{icp_reg} --pcd_tgt {tgt} --pcd_src {src} --h5_file {h5f} --icp_iters {iters} --max_dist {dist}'.format(
            icp_reg=icp_reg, tgt=tgt, src=src, h5f=output_file, iters=ICP_ITERS, dist=ICP_MAX_DIST)
    print cmd
    check_call(cmd, shell=True)


@follows('align_clouds')
def build_static_map():
    pass


def clean():
    for d in dirs:
        print 'deleting %s' % d
        if os.path.exists(d):
            check_call('rm -r %s' % d, shell=True)


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
        'clean': clean
    }

    for key in tasks:
        if key in CMDS:
            tasks[key]()
