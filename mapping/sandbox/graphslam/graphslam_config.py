import os
import json
from os.path import dirname, join as pjoin
import datetime

GRAPHSLAM_PATH = dirname(os.path.abspath(__file__))
GRAPHSLAM_OUT_DIR = '/scail/group/deeplearning/driving_data/zxie/graphslam'
GRAPHSLAM_MATCH_DIR = '%s/matches' % GRAPHSLAM_OUT_DIR
MATCHES_FILE = '%s/matches.json' % GRAPHSLAM_MATCH_DIR
GRAPHSLAM_OPT_POS_DIR = '%s/opt_pos' % GRAPHSLAM_OUT_DIR
GRAPHSLAM_ALIGN_DIR = '%s/align' % GRAPHSLAM_OUT_DIR
GRAPHSLAM_CHUNK_DIR = '%s/chunks' % GRAPHSLAM_OUT_DIR
GRAPHSLAM_LANES_DIR = '%s/lanes' % GRAPHSLAM_OUT_DIR
GRAPHSLAM_VIDEOS_DIR = '%s/videos' % GRAPHSLAM_OUT_DIR

GRAPHSLAM_DIRS = [GRAPHSLAM_OUT_DIR, GRAPHSLAM_MATCH_DIR,
        GRAPHSLAM_OPT_POS_DIR, GRAPHSLAM_ALIGN_DIR, GRAPHSLAM_CHUNK_DIR,
        GRAPHSLAM_LANES_DIR, GRAPHSLAM_VIDEOS_DIR]
for p in GRAPHSLAM_DIRS:
    if not os.path.exists(p):
        os.mkdir(p)

FREEWAY = '280N'
DATE_RANGE = [datetime.date(2014, 4, 3), datetime.date(2014, 4, 29)]

seen_gps_files = set()
GPS_FILES = list()
RSS_LIST = list()
MATCH_JSON_DATA = dict()
if os.path.exists(MATCHES_FILE):
    MATCH_JSON_DATA = json.load(open(MATCHES_FILE, 'r'))['matches']
    for d in MATCH_JSON_DATA:
        if d['gps_file1'] not in seen_gps_files:
            GPS_FILES.append(d['gps_file1'])
            RSS_LIST.append(d['rss1'])
        seen_gps_files.add(d['gps_file1'])
        if d['gps_file2'] not in seen_gps_files:
            GPS_FILES.append(d['gps_file2'])
            RSS_LIST.append(d['rss2'])
        seen_gps_files.add(d['gps_file2'])

# Numerical parameters

GPS_MATCH_DIST_TOL = 20.0
GPS_BBOX_OVERLAP_PADDING = 5.0

# pipeline_config EXPORT_STEP of 5 => 10Hz
# Chunk of 10 => 1s of data
CHUNK_SIZE = 10  # Size of chunks to do ICP with
REALIGN_EVERY = 50  # Compute alignment again every this number of clouds

BIAS_GAMMA = 0.999
dt = 1/50.0

MIN_OVERLAP_THRESH = 0.25
