#!/usr/bin/python
from LaneCorrectorPipeline import Config
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

def get_transforms(folder, run):
    """ Gets the IMU transforms for a run """
    gps_reader = GPSReader(folder + run + '_gpsmark1.out')
    gps_data = gps_reader.getNumericData()
    gps_times = utc_from_gps_log_all(gps_data)
    imu_transforms = IMUTransforms(gps_data)
    return imu_transforms, gps_times

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

    for day in sys.argv[1:]:
        driverseat_folder = '/deep/group/driving/driverseat_data/' + day + '/'
        remote_folder = '/deep/group/driving_data/jkiske/data/' + day + '/'
        q50_data_folder = '/deep/group/driving_data/q50_data/' + day + '/'

        unsafe_mkdir(driverseat_folder)

        config_file_name = driverseat_folder + '/progress.ini'
        configurator = Config(config_file_name)

        configurator.get('organized')
        configurator.get('maps_json')
        configurator.get('radar')
        configurator.get('gps_json')

        print configurator.config

        if configurator.config['organized'] == False:
            for remote_run in sorted(glob.glob(remote_folder + '*/')):
                run = remote_run.split('/')[-2]
                driverseat_run = driverseat_folder + run + '/'
                unsafe_mkdir(driverseat_run)

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
                    print '\tExporting map JSON...'
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

        if configurator.config['radar'] == False:
            for radar_folder in sorted(glob.glob(q50_data_folder + '*_radar/')):
                run = radar_folder.split('/')[-2][:-6] # Remove '_radar'
                radar_data = []
                driverseat_run = driverseat_folder + run + '/'
                json_name = 'radar.json'
                json_radar_name = driverseat_run + json_name
                zip_radar_name = json_radar_name + '.zip'
                print driverseat_run
                if not os.path.isfile(json_radar_name):
                    print '\tExporting radar JSON...'
                    for radar_file_name in sorted(glob.glob(radar_folder + '*')):
                        rdr = loadRDR(radar_file_name)
                        time = radar_file_name.split('/')[-1][:-4] # Remove '.rdr'
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

            configurator.set('radar', True)

        if configurator.config['gps_json'] == False:
            for remote_run in sorted(glob.glob(remote_folder + '*/')):
                run = remote_run.split('/')[-2]
                driverseat_run = driverseat_folder + run + '/'
                print driverseat_run

                json_name = 'gps.json'
                json_gps_name = driverseat_run + json_name
                zip_gps_name = json_gps_name + '.zip'

                if not os.path.isfile(json_gps_name):
                    print '\tExporting gps JSON...'
                    imu_transforms, gps_times = get_transforms(driverseat_run, run)
                    time_data = gps_times.tolist()
                    gps_data = []
                    for imu_xform in imu_transforms:
                        gps_data.append({'rot': quaternion_from_matrix(imu_xform).tolist(),
                                         'pos': imu_xform[:3, 3].tolist()})

                    with open(json_gps_name, 'w') as json_gps_file:
                        json.dump(time_data, json_gps_file)
                        json.dump(gps_data, json_gps_file)

                if not os.path.isfile(zip_gps_name):
                    print '\tZipping...'
                    data = '\n'.join([x for x in open(json_gps_name,
                                                      'r').readlines()])
                    json_zip = ZipFile(zip_gps_name, 'w', ZIP_DEFLATED)
                    json_zip.writestr(json_name, data)
                    json_zip.close()

            configurator.set('maps_json', True)
