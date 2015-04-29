from Q50_config import LoadParameters

def parse_args(folder, video_file):
    """
    Folder is the top level folder.
    video_file is the name of the camera split to use
    ex: parse_args('data/280S', '280S_a2.avi')
    returns a dictionary with keys:
    ('map', 'frames', 'radar' 'gps', 'video', 'params')
    corresponding to file names.
    """

    basename = video_file[0:video_file.rfind('_') + 2]
    fullname = folder + '/' + basename

    map_file = fullname + '.map'
    lidar_folder = fullname + '_frames/'
    radar_folder = fullname + '_radar/'
    gps_file = fullname + '_gps.out'
    gps_mark1_file = fullname + '_gpsmark1.out'
    gps_mark2_file = fullname + '_gpsmark2.out'
    mbly_obj_file = fullname + '_mbly.objproto'
    mbly_lane_file = fullname + '_mbly.lanesproto'
    mbly_ref_file = fullname + '_mbly.refproto'

    video_file_num = video_file.replace(basename, '')
    video_file_num = video_file_num.replace('.avi', '').replace('.zip', '')

    video_file = folder + '/' + video_file
    param_file = folder + '/params.ini'
    params = open(param_file, 'r').readline().rstrip()

    return {'map': map_file,
            'frames': lidar_folder,
            'radar': radar_folder,
            'gps': gps_file,
            'gps_mark1': gps_mark1_file,
            'gps_mark2': gps_mark2_file,
            'mbly_obj': mbly_obj_file,
            'mbly_lanes': mbly_lane_file,
            'mbly_ref_pts': mbly_ref_file,
            'video': video_file,
            'fullname': fullname,
            'basename': basename,
            'cam_num': int(video_file_num),
            'params': LoadParameters(params)}
