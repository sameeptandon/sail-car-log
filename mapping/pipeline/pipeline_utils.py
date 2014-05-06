import os
from subprocess import check_call
from pipeline_config import DATA_DIR, CAMERA

def file_num(file_path):
    return int(os.path.splitext(os.path.basename(file_path))[0])


def print_and_call(cmd):
    print cmd
    check_call(cmd, shell=True)


def dset_dir_from_rss(rss):
    dset = '%s_%s%d' % (rss[1], rss[2], CAMERA)
    return '%s/%s/%s' % (DATA_DIR, rss[0], dset)

def touchf(f):
    print_and_call('touch %s' % f)
