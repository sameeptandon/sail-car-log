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
    
    if video_file[-5] == '1':
        opposite_video = '2'
    else:
        opposite_video = '1'

    opposite_video_file = basename + opposite_video + '.avi'
    video_file = folder + '/' + video_file
    param_file = folder + '/params.ini'
    params = open(param_file, 'r').readline().rstrip()

    return {'map': map_file,
            'frames': lidar_folder,
            'radar': radar_folder,
            'gps': gps_file,
            'video': video_file,
            'opposite_video': opposite_video_file,
            'cam_num': int(video_file[-5]),
            'params': LoadParameters(params)}

