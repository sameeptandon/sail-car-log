from Kitti_config import LoadParameters

def parse_args(folder, basename):
    """
    Folder is the top level folder.
    """
    fullname = folder + '/' + basename
    lidar_folder = fullname + '/velodyne_points/'
    gps_mark1_folder = fullname + '/oxts/'

    return {'frames': lidar_folder,
            'gps_mark1': gps_mark1_folder,
            'fullname': fullname,
            'basename': basename,
            'params': LoadParameters()}
