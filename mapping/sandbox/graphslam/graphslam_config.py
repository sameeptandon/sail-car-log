import os
from os.path import dirname, join as pjoin
from pipeline_config import DATA_DIR

GRAPHSLAM_PATH = dirname(os.path.abspath(__file__))
GRAPHSLAM_OUT_DIR = '%s/graphslam' % DATA_DIR
if not os.path.exists(GRAPHSLAM_OUT_DIR):
    os.mkdir(GRAPHSLAM_OUT_DIR)

# 4-3-14-gilroy / to_gilroy / a
# 4-2-14-monterey / 280S / a
# Segment to align to
SEGMENT_TGT = 'to_gilroy_a2'
# Segment to transform
SEGMENT_SRC = '280S_a2'

TGT_DIR = '%s/%s' % (DATA_DIR, SEGMENT_TGT)
SRC_DIR = '%s/%s' % (DATA_DIR, SEGMENT_SRC)

# GPS trace data and matching
GPS_TGT = '%s/%s_gps.out' % (TGT_DIR, SEGMENT_TGT[:-1])
GPS_SRC = '%s/%s_gps.out' % (SRC_DIR, SEGMENT_SRC[:-1])
GPS_MATCH_DIST_TOL = 5.0  # PARAM
GPS_MATCH_FILE = '%s/%s_match.h5' % (GRAPHSLAM_OUT_DIR, SEGMENT_TGT + '+' + SEGMENT_SRC)
GPS_ALIGNMENT_FILE = '%s/%s_align.h5' % (GRAPHSLAM_OUT_DIR, SEGMENT_TGT + '+' + SEGMENT_SRC)

OPT_POS_FILE_TGT = '%s/%s_opt.npz' % (TGT_DIR, SEGMENT_TGT[:-1])
OPT_POS_FILE_SRC = '%s/%s_opt.npz' % (SRC_DIR, SEGMENT_SRC[:-1])

# TODO Chunk scans for matching...
# pipeline_config EXPORT_STEP of 5 => 10Hz
# Chunk of 10 => 1s of data
CHUNK_SIZE = 10

BIAS_GAMMA = 0.999
dt = 1/50.0
