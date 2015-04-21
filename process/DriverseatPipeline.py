#!/usr/bin/python
from LaneCorrectorPipelineOV import Config
import os
import subprocess
import sys
import shlex
import shutil
import os.path
import numpy as np
import json
import glob
from zipfile import ZipFile, ZIP_DEFLATED
from RadarTransforms import loadRDR
from transformations import quaternion_from_matrix
from GPSReader import GPSReader
from GPSTransforms import IMUTransforms
from LidarTransforms import  utc_from_gps_log_all
from LaneMarkingHelper import mk2_to_mk1
import bisect
from VideoReader import VideoReader
import cv2

def get_transforms(folder, run, mark = 'mark1'):
    """ Gets the IMU transforms for a run """
    gps_reader = GPSReader(folder + run + '_gps' + mark + '.out')
    gps_data = gps_reader.getNumericData()
    gps_times = utc_from_gps_log_all(gps_data)
    imu_transforms = IMUTransforms(gps_data)
    return imu_transforms, gps_times

def mk2_to_radar(mk2_idx, radar_times, gps_times_mk2):
    t = gps_times_mk2[mk2_idx]
    radar_idx = bisect.bisect(radar_times, t) - 1
    return radar_idx

def unsafe_mkdir(name):
    try:
        os.mkdir(name)
    except OSError:
        pass

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print """Usage:
        DriverseatPipeline.py run ...
        """
        sys.exit(-1)

    sub_samp = 2
    for day in sys.argv[1:]:
        driverseat_folder = '/deep/group/driving/driverseat_data/' + day + '/'
        remote_folder = '/deep/group/driving_data/jkiske/data/' + day + '/'
        q50_data_folder = '/deep/group/driving_data/q50_data/' + day + '/'

        unsafe_mkdir(driverseat_folder)

        config_file_name = driverseat_folder + '/progress.ini'
        configurator = Config(config_file_name)

        configurator.get('organized')
        configurator.get('maps_json')
        configurator.get('radar_json')
        configurator.get('gps_json')
        configurator.get('lanes_json')
        configurator.get('planes_json')
        configurator.get('lanes_done_json')
        configurator.get('video_to_mpeg')

        print configurator.config

        if configurator.config['organized'] == False:
            for remote_run in sorted(glob.glob(remote_folder + '*/')):
                run = remote_run.split('/')[-2]
                driverseat_run = driverseat_folder + run + '/'
                unsafe_mkdir(driverseat_run)
            configurator.set('organized', True)

        if configurator.config['maps_json'] == False:
            for remote_run in sorted(glob.glob(remote_folder + '*/')):
                run = remote_run.split('/')[-2]
                driverseat_run = driverseat_folder + run + '/'
                print driverseat_run

                npz_map_name = remote_run + run + '_bg.npz'
                json_name = 'map.json'
                json_map_name = driverseat_run + json_name
                zip_map_name = json_map_name + '.zip'

                if not os.path.isfile(json_map_name):
                    print '\tExporting ' + json_name
                    data = np.load(npz_map_name)['data']
                    # Only includes x,y,z data for the maps
                    json.dump(data[:, :3].tolist(), open(json_map_name, 'w'))

                if not os.path.isfile(zip_map_name):
                    print '\tZipping...'
                    data = '\n'.join([x for x in open(json_map_name,
                                                      'r').readlines()])
                    json_zip = ZipFile(zip_map_name, 'w', ZIP_DEFLATED)
                    json_zip.writestr(json_name, data)
                    json_zip.close()

            configurator.set('maps_json', True)

        if configurator.config['radar_json'] == False:
            for radar_folder in sorted(glob.glob(q50_data_folder + '*_radar/')):
                print radar_folder
                run = radar_folder.split('/')[-2][:-6] # Remove '_radar'
                remote_run = remote_folder + '/' + run + '/'
                driverseat_run = driverseat_folder + run + '/'
                if not os.path.isdir(driverseat_run):
                    continue
                imu_transforms, gps_times_mk1 = get_transforms(remote_run,
                                                               run, 'mark1')
                _, gps_times_mk2 = get_transforms(remote_run, run, 'mark2')
                radar_data = []
                json_name = 'radar.json'
                json_radar_name = driverseat_run + json_name
                zip_radar_name = json_radar_name + '.zip'
                print driverseat_run
                if not os.path.isfile(json_radar_name):
                    print '\tExporting ' + json_name
                    radar_times = [int(f.split('/')[-1][:-4]) for
                                   f in sorted(glob.glob(radar_folder + '*'))]
                    for i in xrange(0, gps_times_mk2.shape[0], sub_samp):
                        radar_idx = mk2_to_radar(i, radar_times, gps_times_mk2)
                        time = radar_times[radar_idx]
                        radar_file = radar_folder + str(time) + '.rdr'
                        rdr = loadRDR(radar_file)
                        data = {time:{'O':rdr[0].tolist(), 'T':rdr[1].tolist()}}
                        radar_data.append(data)
                    json.dump(radar_data, open(json_radar_name, 'w'))

                if not os.path.isfile(zip_radar_name):
                    print '\tZipping...'
                    data = '\n'.join([x for x in open(json_radar_name,
                                                      'r').readlines()])
                    json_zip = ZipFile(zip_radar_name, 'w', ZIP_DEFLATED)
                    json_zip.writestr(json_name, data)
                    json_zip.close()

            configurator.set('radar_json', True)

        if configurator.config['gps_json'] == False:
            for remote_run in sorted(glob.glob(remote_folder + '*/')):
                run = remote_run.split('/')[-2]
                driverseat_run = driverseat_folder + run + '/'
                imu_transforms, gps_times_mk1 = get_transforms(remote_run,
                                                               run, 'mark1')
                _, gps_times_mk2 = get_transforms(remote_run, run, 'mark2')
                print driverseat_run

                json_name = 'gps.json'
                json_gps_name = driverseat_run + json_name
                zip_gps_name = json_gps_name + '.zip'

                if not os.path.isfile(json_gps_name):
                    print '\tExporting ' + json_name
                    gps_data = []
                    for i in xrange(0, gps_times_mk2.shape[0], sub_samp):
                        mk1_i = mk2_to_mk1(i, gps_times_mk1, gps_times_mk2)
                        imu_xform = imu_transforms[mk1_i, :, :]
                        gps_data.append(imu_xform.tolist())

                    with open(json_gps_name, 'w') as json_gps_file:
                        json.dump(gps_data, json_gps_file)

                if not os.path.isfile(zip_gps_name):
                    print '\tZipping...'
                    data = '\n'.join([x for x in open(json_gps_name,
                                                      'r').readlines()])
                    json_zip = ZipFile(zip_gps_name, 'w', ZIP_DEFLATED)
                    json_zip.writestr(json_name, data)
                    json_zip.close()

            configurator.set('gps_json', True)

        if configurator.config['lanes_json'] == False:
            for remote_run in sorted(glob.glob(remote_folder + '*/')):
                run = remote_run.split('/')[-2]
                driverseat_run = driverseat_folder + run + '/'
                lane_file = remote_run + 'multilane_points_planar.npz'

                if not os.path.isfile(lane_file):
                    continue

                print driverseat_run

                json_name = 'lanes.json'
                json_lanes_name = driverseat_run + json_name
                zip_lanes_name = json_lanes_name + '.zip'

                if not os.path.isfile(json_lanes_name):
                    print '\tExporting ' + json_name
                    lanes = np.load(lane_file)
                    num_lanes = lanes['num_lanes']
                    lane_data = {}
                    for i in xrange(num_lanes):
                        lane_data[str(i)] = lanes['lane' + str(i)].tolist()

                    plane_data = lanes['planes'].tolist()

                    with open(json_lanes_name, 'w') as json_lanes_file:
                        json.dump(lane_data, json_lanes_file)

                if not os.path.isfile(zip_lanes_name):
                    print '\tZipping...'
                    data = '\n'.join([x for x in open(json_lanes_name,
                                                      'r').readlines()])
                    json_zip = ZipFile(zip_lanes_name, 'w', ZIP_DEFLATED)
                    json_zip.writestr(json_name, data)
                    json_zip.close()

                # Move the created lanes files to a lanes folder
                unsafe_mkdir(driverseat_run + 'lanes')
                shutil.move(zip_lanes_name, driverseat_run + 'lanes/')

            configurator.set('lanes_json', True)

        if configurator.config['planes_json'] == False:
            for remote_run in sorted(glob.glob(remote_folder + '*/')):
                run = remote_run.split('/')[-2]
                driverseat_run = driverseat_folder + run + '/'
                lane_file = remote_run + 'multilane_points_planar.npz'

                if not os.path.isfile(lane_file):
                    continue

                print driverseat_run

                json_name = 'planes.json'
                json_planes_name = driverseat_run + json_name
                zip_planes_name = json_planes_name + '.zip'

                if not os.path.isfile(json_planes_name):
                    print '\tExporting ' + json_name
                    lanes = np.load(lane_file)
                    plane_data = lanes['planes'].tolist()

                    with open(json_planes_name, 'w') as json_planes_file:
                        json.dump(plane_data, json_planes_file)

                if not os.path.isfile(zip_planes_name):
                    print '\tZipping...'
                    data = '\n'.join([x for x in open(json_planes_name,
                                                      'r').readlines()])
                    json_zip = ZipFile(zip_planes_name, 'w', ZIP_DEFLATED)
                    json_zip.writestr(json_name, data)
                    json_zip.close()

            configurator.set('planes_json', True)

        if configurator.config['lanes_done_json'] == False:
            for remote_run in sorted(glob.glob(remote_folder + '*/')):
                run = remote_run.split('/')[-2]
                driverseat_run = driverseat_folder + run + '/'
                done_folder = remote_folder.\
                              replace('/data/', '/multilane_done/') + run + '/'
                done_lane_files = glob.glob(done_folder + '*done*')
                if len(done_lane_files) == 0:
                    continue
                done_lane_file = done_lane_files[0]

                print driverseat_run

                json_name = 'lanes_done.json'
                json_lanes_name = driverseat_run + json_name
                zip_lanes_name = json_lanes_name + '.zip'

                if not os.path.isfile(json_lanes_name):
                    print '\tExporting ' + json_name
                    lanes = np.load(done_lane_file)
                    num_lanes = lanes['num_lanes']
                    lane_data = {}
                    for i in xrange(num_lanes):
                        lane_data[str(i)] = lanes['lane' + str(i)].tolist()

                    with open(json_lanes_name, 'w') as json_lanes_file:
                        json.dump(lane_data, json_lanes_file)

                if not os.path.isfile(zip_lanes_name):
                    print '\tZipping...'
                    data = '\n'.join([x for x in open(json_lanes_name,
                                                      'r').readlines()])
                    json_zip = ZipFile(zip_lanes_name, 'w', ZIP_DEFLATED)
                    json_zip.writestr(json_name, data)
                    json_zip.close()

                unsafe_mkdir(driverseat_run + 'lanes')
                shutil.move(zip_lanes_name, driverseat_run + 'lanes/')

            configurator.set('lanes_done_json', True)

        fourcc = cv2.cv.CV_FOURCC(*'MPG1')
        if configurator.config['video_to_mpeg'] == False:
            for remote_run in sorted(glob.glob(remote_folder + '*/')):
                run = remote_run.split('/')[-2]
                for video in sorted(glob.glob(remote_run+'/'+run+'60*.zip')):
                    print '\tReading', video
                    driverseat_run = driverseat_folder + run + '/'
                    # If it is a zipped file, remove the .zip extension
                    vid_num = video.replace('zip', '')[-4:-1]
                    # First create an uncompressed mpg from the images. Give
                    # this a file extension mp so we know the process is
                    # not finished
                    mp_vid_name = driverseat_run + 'cam_' + vid_num + \
                                  '_uncompressed.mpg'
                    # The finished file should have the extension mpg
                    mpg_vid_name = mp_vid_name.replace('_uncompressed', '')
                    if not os.path.isfile(mp_vid_name) and \
                       not os.path.isfile(mpg_vid_name):
                        with VideoReader(video) as reader:
                            print '\tWriting ' + mp_vid_name

                            writer = None
                            while True:
                                (succ, I) = reader.getNextFrame()
                                if not succ:
                                    break
                                if reader.framenum % sub_samp == 0:
                                    if I == None:
                                        # This is a corrupted jpeg
                                        prev_fnum = reader.framenum
                                        # Read the next image in the file
                                        (succ, I) = reader.getNextFrame()
                                        if not succ:
                                            break
                                        # Reset the frame number
                                        reader.setFrame(prev_fnum)
                                    I = cv2.pyrDown(cv2.pyrDown(I))
                                    if writer == None:
                                        writer = cv2.VideoWriter(mp_vid_name,
                                                                 fourcc,
                                                                 50, (I.shape[1],
                                                                      I.shape[0]))
                                    writer.write(I)
                    if not os.path.isfile(mpg_vid_name):
                        print '\tCompressing ' + mp_vid_name + ' to ' + mpg_vid_name
                        avconv_cmd = "avconv -i {mp} -f mpeg1video -bf 0 " + \
                                     "-b:v 10M -q:v 7 -r 50 -v quiet {mpg}"
                        avconv_cmd = avconv_cmd.format(mp=mp_vid_name, mpg=mpg_vid_name)
                        subprocess.call(avconv_cmd.split())
                        os.remove(mp_vid_name)

            configurator.set('video_to_mpeg', True)
