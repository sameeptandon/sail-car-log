import os
from os.path import join as pjoin
import numpy as np
import h5py
from gps_viewer import read_gps_fields
from WGS84toENU import WGS84toECEF, WGS84toENU
from pipeline_config import EXPORT_STEP, EXPORT_START, MAPPING_PATH, ICP_ITERS, ICP_MAX_DIST, NUM_CPUS
from graphslam_config import MATCH_JSON_DATA, CHUNK_SIZE, GRAPHSLAM_CHUNK_DIR, GRAPHSLAM_ALIGN_DIR, REALIGN_EVERY
from pipeline_utils import print_and_call, dset_dir_from_rss
from joblib import Parallel, delayed

'''
For every alignment, we need to create two small chunks of the
full maps that we want to align. These chunks are selected by
looking at the best NN matches.

We compute a new alignment every REALIGN_EVERY steps

Since the chunks are stored relative to IMU 0, we need to first
transform them by the global position of IMU 0
'''

def get_ecef0(gps_file):
    llh = read_gps_fields(gps_file, ['lat', 'long', 'height'])
    llh = np.array(llh, dtype=np.float64).T
    ecef = WGS84toECEF(llh)
    return ecef[:, 0]


def get_enu0(gps_file, gps_ref_file):
    llh = read_gps_fields(gps_file, ['lat', 'long', 'height'])
    llh = np.array(llh, dtype=np.float64).T
    llh_ref = read_gps_fields(gps_ref_file, ['lat', 'long', 'height'])
    llh_ref = np.array(llh_ref, dtype=np.float64).T
    return WGS84toENU(llh_ref[0, :], llh)[:, 0]


def vtk_filename(pcd_file):
    return os.path.splitext(pcd_file)[0] + '.vtk'


# Helper function for chunk_and_align all
def chunk_and_align(start1, start2, enu1, enu2, rss1, rss2, pcd_dir1, pcd_dir2, chunk_num, debug=False):
    chunk1_files = list()
    chunk2_files = list()
    for k in range(0, CHUNK_SIZE):
        ind1 = start1 + k
        chunk1_files.append('%s/%d.pcd' % (pcd_dir1, ind1))
        assert os.path.exists(chunk1_files[-1])
        ind2 = start2 + k
        chunk2_files.append('%s/%d.pcd' % (pcd_dir2, ind2))
        assert os.path.exists(chunk2_files[-1])

    merged_chunks1 = '%s/%s' % (GRAPHSLAM_CHUNK_DIR, '--'.join(rss1) + '+' + '--'.join(rss2) + '%d_1.pcd' % chunk_num)
    merged_chunks2 = '%s/%s' % (GRAPHSLAM_CHUNK_DIR, '--'.join(rss1) + '+' + '--'.join(rss2) + '%d_2.pcd' % chunk_num)

    # Concatenate

    cmd = 'concatenate_points_pcd %s %s' % (' '.join(chunk1_files), merged_chunks1)
    print_and_call(cmd)

    cmd = 'concatenate_points_pcd %s %s' % (' '.join(chunk2_files), merged_chunks2)
    print_and_call(cmd)

    # Translate

    cmd = 'transform_point_cloud %s %s -trans %f,%f,%f' % (merged_chunks1, merged_chunks1, enu1[0]-enu2[0], enu1[1]-enu2[1], enu1[2]-enu2[2])
    print_and_call(cmd)
    #cmd = 'transform_point_cloud %s %s -trans %f,%f,%f' % (merged_chunks2, merged_chunks2, -1*enu2[0], -1*enu2[1], -1*enu2[2])
    #print_and_call(cmd)

    # Generate VTK files so we can easily visualize to debug

    if debug:
        cmd = 'pcl_pcd2vtk %s %s' % (merged_chunks1, vtk_filename(merged_chunks1))
        print_and_call(cmd)
        cmd = 'pcl_pcd2vtk %s %s' % (merged_chunks2, vtk_filename(merged_chunks2))
        print_and_call(cmd)

    # Finally perform alignment

    reg = '%s/bin/align_clouds' % MAPPING_PATH
    h5f = '%s/%s' % (GRAPHSLAM_ALIGN_DIR, '--'.join(rss1) + '+' + '--'.join(rss2) + '--%d' % chunk_num + '.h5')

    cmd = '{reg} --pcd_tgt {tgt} --pcd_src {src} --h5_file {h5f} --icp_iters {iters} --max_dist {dist}'.format(
            reg=reg, tgt=merged_chunks1, src=merged_chunks2, h5f=h5f, iters=ICP_ITERS, dist=ICP_MAX_DIST)
    print_and_call(cmd)


def get_closest_key_value(k, d, max_shift=5):
    shift = -1
    while k not in d:
        k = k + shift
        shift = -1 * (abs(shift) + 1) * cmp(shift, 0)
        assert abs(shift) < max_shift, 'Index %d shift %d' % (k, shift)
    return d[k]


def chunk_and_align_all(d):
    print d['match_file']
    rss1 = d['rss1']
    rss2 = d['rss2']
    pcd_dir1 = pjoin(dset_dir_from_rss(rss1), 'pcd_downsampled_normals')
    pcd_dir2 = pjoin(dset_dir_from_rss(rss2), 'pcd_downsampled_normals')

    # Read and save initial transform files
    enu1 = get_enu0(d['gps_file1'], d['gps_file1'])
    enu2 = get_enu0(d['gps_file2'], d['gps_file1'])

    h5f = h5py.File(d['match_file'], 'r')
    nn_matches = h5f['matches'][...]
    nn_dict = dict(zip(nn_matches[:, 1], nn_matches[:, 0]))
    h5f.close()

    assert EXPORT_START == 0
    start1 = nn_matches[0, 1] / EXPORT_STEP

    args_all = list()
    chunk_num = 0
    for k in range(start1, nn_matches[-1, 1] / EXPORT_STEP - CHUNK_SIZE, REALIGN_EVERY):
        #def chunk_and_align(start1, start2, enu1, enu2, rss1, rss2, pcd_dir1, pcd_dir2, chunk_num):
        try:
            k2 = get_closest_key_value(k * EXPORT_STEP, nn_dict, max_shift=10)
        except:
            # TODO Think this sometimes occurs near end of alignments
            break
        args_all.append((k, k2 / EXPORT_STEP, enu1, enu2, rss1, rss2, pcd_dir1, pcd_dir2, chunk_num))
        chunk_num += 1

    Parallel(n_jobs=NUM_CPUS)(delayed(chunk_and_align)(*args) for args in args_all)
    # For debugging
    '''
    for args in args_all:
        _, _, _, _, rss1, rss2, _, _, chunk_num = args
        h5f = '%s/%s' % (GRAPHSLAM_ALIGN_DIR, '--'.join(rss1) + '+' + '--'.join(rss2) + '--%d' % chunk_num + '.h5')
        if os.path.exists(h5f):
            continue
        chunk_and_align(*args)
    '''


if __name__ == '__main__':
    for d in MATCH_JSON_DATA:
        chunk_and_align_all(d)
