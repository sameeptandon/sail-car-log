from ruffus import follows, transform, regex, mkdir,\
        pipeline_printout, pipeline_printout_graph,\
        pipeline_run, files, merge,\
        touch_file, posttask, jobs_limit
import os
import sys
from subprocess import check_call
from pipeline_config import POINTS_H5_DIR,\
        PCD_DIR, PCD_DOWNSAMPLED_DIR, NUM_CPUS, DSET_DIR, DSET,\
        SAIL_CAR_LOG_PATH, CLOUDPROC_PATH, DOWNSAMPLE_LEAF_SIZE,\
        K_NORM_EST, PCD_DOWNSAMPLED_NORMALS_DIR, ICP_TRANSFORMS_DIR,\
        ICP_ITERS, ICP_MAX_DIST, REMOTE_DATA_DIR, REMOTE_FILES,\
        EXPORT_FULL, GPS_FILE, MAP_FILE, COLOR_DIR, COLOR_CLOUDS_DIR,\
        MERGED_CLOUDS_DIR, MAP_COLOR_WINDOW, OCTOMAP_DIR, OCTOMAP_RES,\
        EXPORT_NUM, COLOR_OCTOMAP_DIR, COLOR_OCTOMAP_RES, OCTOMAP_FILE,\
        COLOR_OCTOMAP_FILE, COLOR_OCTOMAP_BT, COLOR_OCTOMAP_MESH, MERGED_CLOUD_FILE,\
        CAST_OCTOMAP_SINGLE, MERGED_VTK_FILE
from pipeline_utils import file_num

# TODO Use generate_frames_and_map.py

dirs = [POINTS_H5_DIR, PCD_DIR, PCD_DOWNSAMPLED_DIR,
        PCD_DOWNSAMPLED_NORMALS_DIR, ICP_TRANSFORMS_DIR, COLOR_DIR,
        COLOR_CLOUDS_DIR, MERGED_CLOUDS_DIR, OCTOMAP_DIR, COLOR_OCTOMAP_DIR]
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


#@follows("download_files")
@follows(*MKDIRS)
@files('./params.ini', './params.h5')
def convert_params_to_h5(input_file, output_file):
    converter = '%s/cloudproc/pipeline/params_to_h5.py' % SAIL_CAR_LOG_PATH
    cmd = 'python %s' % converter
    check_call(cmd, shell=True)


#@follows("download_files")
@follows(*MKDIRS)
@files('params.ini', '%s/sentinel' % POINTS_H5_DIR)
@posttask(touch_file('%s/sentinel' % POINTS_H5_DIR))
def convert_ldr_to_h5(dummy_file, output_file):
    if os.path.exists('%s/sentinel' % POINTS_H5_DIR):
        return
    exporter = '%s/cloudproc/pipeline/ldr_to_h5.py' % SAIL_CAR_LOG_PATH
    cmd = 'python {exporter} {fgps} {fmap} {h5_dir}'.format(exporter=exporter, fgps=GPS_FILE, fmap=MAP_FILE, h5_dir=POINTS_H5_DIR)
    if EXPORT_FULL:
        cmd += ' --full'
    check_call(cmd, shell=True)


@follows('convert_ldr_to_h5')
@transform('./h5/*.transform',
           regex('./h5/(.*?).transform'),
           r'./h5/\1.euler')
def convert_matrix_to_euler(input_file, output_file):
    converter = '%s/cloudproc/pipeline/matrix_to_euler.py' % SAIL_CAR_LOG_PATH
    cmd = 'python %s %s %s' % (converter, input_file, output_file)
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
@jobs_limit(1)
@merge(convert_h5_to_pcd, OCTOMAP_FILE)
def build_octomap(input_files, output_file):
    cmd = '{0}/bin/build_octomap'.format(CLOUDPROC_PATH)
    if CAST_OCTOMAP_SINGLE:
        cmd += ' --single'
    print cmd
    check_call(cmd, shell=True)


''' TODO parallelize
@follows('build_octomap')
@transform('./pcd_downsampled/*.pcd',
           regex('./pcd_downsampled/(.*?).pcd'),
           r'./color/\1.h5')
def project_color(input_pcd, output_color_file):
    pass
'''
#@follows('build_octomap')
@jobs_limit(1)
@files(None, '{0}/0.h5'.format(COLOR_DIR))
def project_color(dummy_file, output_file):
    binary = '%s/bin/octomap_color' % CLOUDPROC_PATH
    print binary
    check_call(binary, shell=True)


@follows('project_color')
@transform('./color/*.h5',
           regex('./color/(.*?).h5'),
           r'./color_clouds/\1.pcd',
           r'./pcd_downsampled/\1.pcd')
def color_clouds(color_file, output_file, pcd_file):
    converter = '%s/bin/color_cloud' % CLOUDPROC_PATH
    cmd = '%s %s %s %s' % (converter, pcd_file, color_file, output_file)
    print cmd
    check_call(cmd, shell=True)


@follows('color_clouds')
@jobs_limit(1)
@merge(color_clouds, COLOR_OCTOMAP_FILE)
def build_color_octomap(input_files, output_file):
    cmd = '{0}/bin/build_color_octomap'.format(CLOUDPROC_PATH)
    print cmd
    check_call(cmd, shell=True)


@follows('build_color_octomap')
@files(COLOR_OCTOMAP_BT, COLOR_OCTOMAP_MESH)
def convert_octomap_to_mesh(input_file, output_file):
    pass


@follows('color_clouds')
@merge('./color_clouds/*.pcd', './merged_clouds/merged_%d.pcd' % MAP_COLOR_WINDOW)
def merge_color_clouds(cloud_files, merged_cloud_file):
    files = [f for f in cloud_files if os.path.exists(f)]
    # Concatenate PCD files
    cmd = 'pcl_concatenate_points_pcd ' + ' '.join(files) + '; mv output.pcd %s' % MERGED_CLOUD_FILE
    check_call(cmd, shell=True)
    # Convert merged cloud to vtk for visualizer
    cmd = 'pcl_pcd2vtk %s %s' % (MERGED_CLOUD_FILE, MERGED_VTK_FILE)
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
