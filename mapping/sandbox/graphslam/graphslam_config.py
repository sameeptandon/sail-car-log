import os
import json
from os.path import dirname, join as pjoin
import datetime

FREEWAY = os.getenv('GRAPHSLAM_FREEWAY', '280N')

GRAPHSLAM_PATH = dirname(os.path.abspath(__file__))
#GRAPHSLAM_OUT_DIR = '/scail/group/deeplearning/driving_data/zxie/graphslam'
GRAPHSLAM_OUT_DIR = '/scr/scl/graphslam/%s' % FREEWAY
GRAPHSLAM_MATCH_DIR = '%s/matches' % GRAPHSLAM_OUT_DIR
MATCHES_FILE = '%s/matches.json' % GRAPHSLAM_MATCH_DIR
GRAPHSLAM_OPT_POS_DIR = '%s/opt_pos' % GRAPHSLAM_OUT_DIR
GRAPHSLAM_ALIGN_DIR = '%s/align' % GRAPHSLAM_OUT_DIR
GRAPHSLAM_CHUNK_DIR = '%s/chunks' % GRAPHSLAM_OUT_DIR
GRAPHSLAM_MAPS_DIR = '%s/maps' % GRAPHSLAM_OUT_DIR
GRAPHSLAM_VIDEOS_DIR = '%s/videos' % GRAPHSLAM_OUT_DIR
GRAPHSLAM_EVAL_DIR = '%s/eval' % GRAPHSLAM_OUT_DIR

GRAPHSLAM_DIRS = [GRAPHSLAM_OUT_DIR, GRAPHSLAM_MATCH_DIR,
        GRAPHSLAM_OPT_POS_DIR, GRAPHSLAM_ALIGN_DIR, GRAPHSLAM_CHUNK_DIR,
        GRAPHSLAM_MAPS_DIR, GRAPHSLAM_VIDEOS_DIR, GRAPHSLAM_EVAL_DIR]
for p in GRAPHSLAM_DIRS:
    if not os.path.exists(p):
        os.mkdir(p)

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

# Chunk of 10 => 1s of data
CHUNK_SIZE = 25  # Size of chunks to do ICP with
REALIGN_EVERY = CHUNK_SIZE  # Compute alignment again every this number of clouds

BIAS_GAMMA = 0.999
dt = 1/50.0

MIN_OVERLAP_THRESH = 0.25

MAX_VIDEO_FRAMES = 5000
