from ruffus import follows, transform, regex, mkdir,\
        pipeline_printout, pipeline_printout_graph,\
        pipeline_run, files, merge,\
        touch_file, posttask, jobs_limit
import os
import sys
from subprocess import check_call
from pipeline_config import POINTS_H5_DIR,\
        PCD_DIR, PCD_DOWNSAMPLED_DIR, NUM_CPUS, DSET_DIR, DSET,\
        SAIL_CAR_LOG_PATH, MAPPING_PATH, DOWNSAMPLE_LEAF_SIZE,\
        K_NORM_EST, PCD_DOWNSAMPLED_NORMALS_DIR, ICP_TRANSFORMS_DIR,\
        ICP_ITERS, ICP_MAX_DIST, REMOTE_DATA_DIR, REMOTE_FILES,\
        EXPORT_FULL, GPS_FILE, MAP_FILE, COLOR_DIR, COLOR_CLOUDS_DIR,\
        MERGED_CLOUDS_DIR, MAP_COLOR_WINDOW, OCTOMAP_DIR,\
        COLOR_OCTOMAP_DIR, OCTOMAP_FILE,\
        COLOR_OCTOMAP_FILE, COLOR_OCTOMAP_BT, MERGED_CLOUD_FILE,\
        CAST_OCTOMAP_SINGLE, MERGED_VTK_FILE, STATIC_CLOUD_FILE,\
        STATIC_VTK_FILE, DYNAMIC_CLOUD_FILE, DYNAMIC_VTK_FILE,\
        FILTERED_CLOUDS_DIR, PARAMS_TO_LOAD,\
        MERGED_COLOR_CLOUDS_DIR, MERGED_COLOR_CLOUD_FILE,\
        MERGED_COLOR_VTK_FILE
from pipeline_utils import file_num


dirs = [POINTS_H5_DIR, PCD_DIR, PCD_DOWNSAMPLED_DIR,
        PCD_DOWNSAMPLED_NORMALS_DIR, ICP_TRANSFORMS_DIR, COLOR_DIR,
        COLOR_CLOUDS_DIR, MERGED_CLOUDS_DIR, MERGED_COLOR_CLOUDS_DIR,
        OCTOMAP_DIR, COLOR_OCTOMAP_DIR, FILTERED_CLOUDS_DIR]
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


@follows('download_files')
@files('./%s_gps.bag' % DSET[:-1], '%s_frames' % DSET[:-1])
def generate_frames_and_map(input_file, output_dir):
    cmd = 'cd %s/lidar; python generate_frames.py %s %s; python generate_map.py %s %s; cd -' % (SAIL_CAR_LOG_PATH, DSET_DIR, PARAMS_TO_LOAD, DSET_DIR, PARAMS_TO_LOAD)
    print cmd
    check_call(cmd, shell=True)


@follows('generate_frames_and_map')
@files('./params.ini', './params.h5')
def convert_params_to_h5(input_file, output_file):
    converter = '%s/mapping/pipeline/params_to_h5.py' % SAIL_CAR_LOG_PATH
    cmd = 'python %s' % converter
    check_call(cmd, shell=True)


@follows('convert_params_to_h5')
@files('params.ini', '%s/sentinel' % POINTS_H5_DIR)
@posttask(touch_file('%s/sentinel' % POINTS_H5_DIR))
def convert_ldr_to_h5(dummy_file, output_file):
    if os.path.exists('%s/sentinel' % POINTS_H5_DIR):
        return
    exporter = '%s/mapping/pipeline/ldr_to_h5.py' % SAIL_CAR_LOG_PATH
    cmd = 'python {exporter} {fgps} {fmap} {h5_dir}'.format(exporter=exporter, fgps=GPS_FILE, fmap=MAP_FILE, h5_dir=POINTS_H5_DIR)
    if EXPORT_FULL:
        cmd += ' --full'
    print cmd
    check_call(cmd, shell=True)


@follows('convert_ldr_to_h5')
@transform('%s/*.h5' % POINTS_H5_DIR,
           regex('%s/(.*?).h5' % POINTS_H5_DIR),
           r'%s/\1.pcd' % PCD_DIR)
def convert_h5_to_pcd(input_file, output_file):
    h5_to_pcd = '%s/bin/h5_to_pcd' % MAPPING_PATH
    cmd = '%s --h5 %s --pcd %s' % (h5_to_pcd, input_file, output_file)
    print cmd
    check_call(cmd, shell=True)


@follows('convert_h5_to_pcd')
@transform('%s/*.pcd' % PCD_DIR,
           regex('%s/(.*?).pcd' % PCD_DIR),
           r'%s/\1.pcd' % PCD_DOWNSAMPLED_DIR)
def downsample_pcds(input_file, output_file):
    downsampler = '%s/bin/downsample_cloud' % MAPPING_PATH
    cmd = '%s --src_pcd %s --out_pcd %s --leaf_size %f' % (downsampler, input_file,
                output_file, DOWNSAMPLE_LEAF_SIZE)
    print cmd
    check_call(cmd, shell=True)


@follows('downsample_pcds')
@jobs_limit(1)
@merge(convert_h5_to_pcd, OCTOMAP_FILE)
def build_octomap(input_files, output_file):
    cmd = '{0}/bin/build_octomap'.format(MAPPING_PATH)
    if CAST_OCTOMAP_SINGLE:
        cmd += ('; ' + cmd + ' --single')
    print cmd
    check_call(cmd, shell=True)


''' TODO parallelize
@follows('build_octomap')
@transform('%s/*.pcd' % PCD_DOWNSAMPLED_DIR,
           regex('%s/(.*?).pcd' % PCD_DOWNSAMPLED_DIR),
           r'%s/\1.h5' % COLOR_DIR)
def project_color(input_pcd, output_color_file):
    pass
'''
@follows('build_octomap')
@jobs_limit(1)
@files(None, '{0}/0.h5'.format(COLOR_DIR))
def project_color(dummy_file, output_file):
    binary = '%s/bin/octomap_color' % MAPPING_PATH
    print binary
    check_call(binary, shell=True)


@follows('project_color')
@transform('%s/*.h5' % COLOR_DIR,
           regex('%s/(.*?).h5' % COLOR_DIR),
           r'%s/\1.pcd' % COLOR_CLOUDS_DIR,
           r'%s/\1.pcd' % PCD_DOWNSAMPLED_DIR)
def color_clouds(color_file, output_file, pcd_file):
    converter = '%s/bin/color_cloud' % MAPPING_PATH
    cmd = '%s %s %s %s' % (converter, pcd_file, color_file, output_file)
    print cmd
    check_call(cmd, shell=True)


@follows('color_clouds')
@jobs_limit(1)
@merge(color_clouds, COLOR_OCTOMAP_FILE)
def build_color_octomap(input_files, output_file):
    cmd = '{0}/bin/build_color_octomap'.format(MAPPING_PATH)
    print cmd
    check_call(cmd, shell=True)


@follows('color_clouds')
@transform('%s/*.pcd' % COLOR_CLOUDS_DIR,
           regex('%s/(.*?).pcd' % COLOR_CLOUDS_DIR),
           r'%s/\1_static.pcd' % FILTERED_CLOUDS_DIR,
           r'%s/\1_dynamic.pcd' % FILTERED_CLOUDS_DIR)
def octomap_filter_single(input_file, static_file, dynamic_file):
    cmd = '%s/bin/octomap_filter %s %s %s' % (MAPPING_PATH, input_file, static_file, dynamic_file)
    print cmd
    check_call(cmd, shell=True)
    static_vtk_file = os.path.splitext(static_file)[0] + '.vtk'
    dynamic_vtk_file = os.path.splitext(dynamic_file)[0] + '.vtk'
    cmd = 'pcl_pcd2vtk %s %s; pcl_pcd2vtk %s %s' % (static_file, static_vtk_file, dynamic_file, dynamic_vtk_file)
    print cmd
    check_call(cmd, shell=True)


def chunk(l, n):
    for k in xrange(0, len(l), n):
        yield l[k:k + n]


# FIXME Repeats code in merge_color_clouds
@follows('downsample_pcds')
@merge('%s/*.pcd' % PCD_DOWNSAMPLED_DIR, '%s/merged.pcd' % MERGED_CLOUDS_DIR)
def merge_raw_clouds(cloud_files, merged_cloud_file):
    files = [f for f in cloud_files if os.path.exists(f)]

    # Have to chunk the files since there's limit on number of command line arguments
    chunks = chunk(files, 500)
    merged_chunk_files = list()
    k = 0
    for chunk_files in chunks:
        merged_chunk_file = os.path.dirname(MERGED_CLOUD_FILE) + '/chunk%d.pcd' % k
        # Concatenate PCD files
        cmd = 'concatenate_points_pcd ' + ' '.join(chunk_files) + ' ' + merged_chunk_file
        print cmd
        check_call(cmd, shell=True)
        merged_chunk_files.append(merged_chunk_file)
        k += 1

    cmd = 'concatenate_points_pcd ' + ' '.join(merged_chunk_files) + ' ' + MERGED_CLOUD_FILE
    print cmd
    check_call(cmd, shell=True)

    for chunk_file in merged_chunk_files:
        cmd = 'rm %s' % chunk_file
        print cmd
        check_call(cmd, shell=True)

    # Color the merged cloud by intensity
    cmd = '%s/bin/color_intensity %s %s' % (MAPPING_PATH, MERGED_CLOUD_FILE, MERGED_CLOUD_FILE)
    print cmd
    check_call(cmd, shell=True)

    # Convert merged cloud to vtk for visualizer
    cmd = 'pcl_pcd2vtk %s %s' % (MERGED_CLOUD_FILE, MERGED_VTK_FILE)
    check_call(cmd, shell=True)


@follows('color_clouds')
@merge('%s/*.pcd' % COLOR_CLOUDS_DIR, '%s/merged_%d.pcd' % (MERGED_COLOR_CLOUDS_DIR, MAP_COLOR_WINDOW))
def merge_color_clouds(cloud_files, merged_cloud_file):
    files = [f for f in cloud_files if os.path.exists(f)]

    # Have to chunk the files since there's limit on number of command line arguments
    chunks = chunk(files, 500)
    merged_chunk_files = list()
    k = 0
    for chunk_files in chunks:
        merged_chunk_file = os.path.dirname(MERGED_COLOR_CLOUD_FILE) + '/chunk%d.pcd' % k
        # Concatenate PCD files
        cmd = 'concatenate_points_pcd ' + ' '.join(chunk_files) + ' ' + merged_chunk_file
        print cmd
        check_call(cmd, shell=True)
        merged_chunk_files.append(merged_chunk_file)
        k += 1

    cmd = 'concatenate_points_pcd ' + ' '.join(merged_chunk_files) + ' ' + MERGED_COLOR_CLOUD_FILE
    print cmd
    check_call(cmd, shell=True)

    for chunk_file in merged_chunk_files:
        cmd = 'rm %s' % chunk_file
        print cmd
        check_call(cmd, shell=True)

    # Convert merged cloud to vtk for visualizer
    cmd = 'pcl_pcd2vtk %s %s' % (MERGED_COLOR_CLOUD_FILE, MERGED_COLOR_VTK_FILE)
    check_call(cmd, shell=True)


@follows('merge_color_clouds')
@files(MERGED_COLOR_CLOUD_FILE, STATIC_CLOUD_FILE)
def octomap_filter(input_file, output_file):
    cmd = '%s/bin/octomap_filter %s %s %s' % (MAPPING_PATH, MERGED_COLOR_CLOUD_FILE, STATIC_CLOUD_FILE, DYNAMIC_CLOUD_FILE)
    print cmd
    check_call(cmd, shell=True)
    cmd = 'pcl_pcd2vtk %s %s; pcl_pcd2vtk %s %s' % (STATIC_CLOUD_FILE, STATIC_VTK_FILE, DYNAMIC_CLOUD_FILE, DYNAMIC_VTK_FILE)
    print cmd
    check_call(cmd, shell=True)


@follows('downsample_pcds')
@transform('%s/*.pcd' % PCD_DOWNSAMPLED_DIR,
           regex('%s/(.*?).pcd' % PCD_DOWNSAMPLED_DIR),
           r'%s/\1.pcd' % PCD_DOWNSAMPLED_NORMALS_DIR)
def estimate_normals(input_file, output_file):
    norm_est = '%s/bin/estimate_normals' % MAPPING_PATH
    cmd = '%s --src_pcd %s --out_pcd %s --k %d' % (norm_est, input_file,
                output_file, K_NORM_EST)
    print cmd
    check_call(cmd, shell=True)


@follows('estimate_normals')
@transform('%s/*.pcd' % PCD_DOWNSAMPLED_NORMALS_DIR,
           regex('%s/(.*?).pcd' % PCD_DOWNSAMPLED_NORMALS_DIR),
           r'%s/\1.h5' % ICP_TRANSFORMS_DIR)
def align_clouds(input_file, output_file):
    icp_reg = '%s/bin/align_clouds' % MAPPING_PATH
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


def clean():
    for d in dirs:
        print 'deleting %s' % d
        if os.path.exists(d):
            check_call('rm -r %s' % d, shell=True)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'Usage: python pipeline.py print,graph,run (task1,task2)'
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
