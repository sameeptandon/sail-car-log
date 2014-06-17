import os
import sys
from GPSReader import GPSReader
from VideoReader import VideoReader
from GPSTransforms import IMUTransforms
from LidarTransforms import LDRLoader, utc_from_gps_log_all
from ArgParser import parse_args
from pipeline_config import LDR_DIR
from pipeline_config import EXPORT_START, EXPORT_NUM, EXPORT_STEP

'''
Create new lidar files synced to video
'''


if __name__ == '__main__':
    args = parse_args(sys.argv[1], sys.argv[2])
    cam_num = int(sys.argv[2][-5])
    video_file = args['video']
    video_reader = VideoReader(video_file)
    params = args['params']
    cam = params['cam'][cam_num-1]

    gps_reader_mark2 = GPSReader(args['gps_mark2'])
    gps_data_mark2 = gps_reader_mark2.getNumericData()

    lidar_loader = LDRLoader(args['frames'])
    imu_transforms_mark2 = IMUTransforms(gps_data_mark2)
    gps_times_mark2 = utc_from_gps_log_all(gps_data_mark2)

    frame_ldr_map = ''

    # TODO Check this
    if EXPORT_START != 0:
        video_reader.setFrame(EXPORT_START)

    count = 0
    while count < EXPORT_NUM:
        (success, I) = video_reader.getNextFrame()
        if not success:
            print 'Reached end of video'
            break
        fnum = video_reader.framenum * 2
        #t = utc_from_gps_log(gps_data_mark2[fnum, :])
        if fnum >= gps_times_mark2.size:
            print 'No more GPS transforms'
            break
        t = gps_times_mark2[fnum]
        data, t_data = lidar_loader.loadLDRWindow(t-50000, 0.1)
        print data.shape

        ldrfile = os.path.join(LDR_DIR, '%d.ldr' % (video_reader.framenum - 1))
        data.tofile(ldrfile)

        # Step

        count += 1
        for k in range(EXPORT_STEP - 1):
            (success, I) = video_reader.getNextFrame()
