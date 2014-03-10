def parse_args(folder, video_file):
    """Folder is the top level folder.
       video_file is the name of the camera split to use
       ex: parse_args('data/280S', '280S_a2.avi')
       returns a dictionary with keys: ('map', 'frames', 'gps', 'video') 
       corresponding to file names.
    """

    basename = folder + '/' + video_file[:-5]

    map_file = basename + '.map'
    frames_folder = basename + '_frames/'
    gps_file = basename + '_gps.out'
    video_file = folder + '/' + video_file

    return {'map': map_file,
            'frames': frames_folder,
            'gps': gps_file,
            'video': video_file}


# print parse_args('data/280S', '280S_a2.avi')