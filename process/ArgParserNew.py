from Q50_config import LoadParameters

def parse_args(folder, video_file):
    """Folder is the top level folder.
video_file is the name of the camera split to use
ex: parse_args('data/280S', '280S_a2.avi')
returns a dictionary with keys: ('map', 'frames', 'radar' 'gps', 'video', 'params')
corresponding to file names.
"""

    basename = folder + '/' + video_file[:-5]

    map_file = basename + '.map'
    lidar_folder = basename + '_frames/'
    radar_folder = basename + '_rdr/'
    gps_file = basename + '_gps.out'
    gps_mark1_file = basename + '_gpsmark1.out'
    gps_mark2_file = basename + '_gpsmark2.out'
   
    video_file_num = video_file[-5]

    video_file = folder + '/' + video_file
    param_file = folder + '/params.ini'
    params = open(param_file, 'r').readline().rstrip()

    return {'map': map_file,
            'frames': lidar_folder,
            'radar': radar_folder,
            'gps': gps_file,
            'gps_mark1': gps_mark1_file,
            'gps_mark2': gps_mark2_file,
            'video': video_file,
            'cam_num': int(video_file[-5]),
            'params': LoadParameters(params)}
