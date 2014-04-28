import sys
from subprocess import check_call
from pipeline_config import MAPPING_PATH, ICP_ITERS, ICP_MAX_DIST
#import h5py

if __name__ == '__main__':
    tgt_dir = sys.argv[1]
    src_dir = sys.argv[2]
    gps_match_file = sys.argv[3]
    out_file = sys.argv[4]

    '''
    # FIXME PARAM
    TGT_START = 1700
    SRC_START = 0
    STEP = 5

    h5f = h5py.File(gps_match_file, 'r')
    matches = h5f['matches'][...]
    h5f.close()
    src_ind0 = matches[0, 0]
    tgt_ind0 = matches[0, 1]
    '''

    # FIXME Multiple of 5 issue...
    pcd_tgt_ind = 4
    pcd_src_ind = 6
    #pcd_tgt = '%s/pcd_downsampled_normals/%d.pcd' % (tgt_dir, pcd_tgt_ind)
    #pcd_src = '%s/pcd_downsampled_normals/%d.pcd' % (src_dir, pcd_src_ind)
    pcd_tgt = '%s/pcd_downsampled_normals/output.pcd' % tgt_dir
    pcd_src = '%s/pcd_downsampled_normals/output.pcd' % src_dir

    cmd = '%s/bin/align_clouds --pcd_tgt %s --pcd_src %s --h5_file %s --icp_iters %d --max_dist %f' % (MAPPING_PATH, pcd_tgt, pcd_src, out_file, ICP_ITERS, ICP_MAX_DIST)
    print cmd
    check_call(cmd, shell=True)
