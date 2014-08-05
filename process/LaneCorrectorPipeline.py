#!/usr/bin/python

from ConfigParser import SafeConfigParser
import os
import subprocess
import sys
import shlex
import shutil
import os.path
import glob
from MapBuilder import MapBuilder
from ArgParser import parse_args

class Config:
    def __init__(self, config_name):
        self.config_name = config_name
        self.parser = SafeConfigParser()
        self.config = {}
        self.section = 'progress'

    def exists(self):
        return os.path.isfile(self.config_name)

    def delete(self):
        try:
            os.remove(config_file_name)
        except OSError:
            pass

    def set(self, key, value):
        if not self.exists():
            self.parser.add_section(self.section)
            config_file = open(self.config_name, 'w+')
            self.parser.write(config_file)
            config_file.close()

        with open(self.config_name, 'w') as config_file:
            self.config[key] = value
            self.parser.set('progress', str(key), str(value))
            self.parser.write(config_file)

    def get(self, key, type='bool'):
        if type == 'bool':
            self.parser.read(self.config_name)
            val = self.parser.get(self.section, key) == 'True'
        else:
            raise TypeError

        self.config[key] = val
        return val

if len(sys.argv) < 2:
    print """Usage:
    download.py -d -f folder/
    -d = dry-run
    -f = force"""
    sys.exit(-1)

local_folder = 'data/' + sys.argv[-1]
remote_folder = sys.argv[-1]
try:
    os.mkdir(local_folder)
except OSError:
    pass

config_file_name = local_folder + '/progress.ini'
configurator = Config(config_file_name)

if '-f' in sys.argv:
    configurator.delete()

if configurator.exists():
    configurator.get('downloaded')
    configurator.get('organized')
    configurator.get('map')
    configurator.get('multilane')
else:
    configurator.set('downloaded', False)
    configurator.set('organized', False)
    configurator.set('map', False)
    configurator.set('multilane', False)

print configurator.config

if configurator.config['downloaded'] == False:
    cmd = """rsync --progress -a --prune-empty-dirs --include="*.rdr" \
    --include="*.ldr" --include="*.avi" --include="*.out" \
    --include="params.ini" --include="*lanes.pickle" --filter="-! */" \
    jkiske@gorgon33:~/q50_data/{remote} data/""".format(
        remote=remote_folder)

    tokens = shlex.split(cmd)
    if '-d' in sys.argv:
        tokens.insert(1, '--dry-run')
        print 'DRY RUN'

    subprocess.call(tokens)

    if not '-d' in sys.argv:
        configurator.set('downloaded', True)

if configurator.config['organized'] == False:
    for video in glob.glob(local_folder + '/split_0*1.avi'):
        organized_folder = video.replace('split_0_', '').replace('1.avi', '')
        try:
            os.mkdir(organized_folder)
        except OSError:
            pass

        tokens = organized_folder.split('/')
        tokens[-1] = '*' + tokens[-1] + '*'
        all_file_glob = '/'.join(tokens)

        all_files = glob.glob(all_file_glob)

        folder_name = organized_folder.split('/')[-1]
        for file in all_files:
            file_parts = file.split('/')
            if file_parts[-1] != folder_name:
                file_parts.insert(-1, folder_name)
                old_file = file
                new_file = '/'.join(file_parts)
                print old_file, new_file
                shutil.move(old_file, new_file)

        params = local_folder + '/params.ini'
        folder_params = local_folder + '/' + folder_name + '/params.ini'
        shutil.copy(params, folder_params)

    configurator.set('organized', True)

if configurator.config['map'] == False:
    for run in glob.glob(local_folder + '/*/'):
        if os.path.isdir(run):
            base = run.split('/')[-2]
            temp_vid = base + '2.avi'
            args = parse_args(run, temp_vid)
            mb = MapBuilder(args, 1, 600, 0.5, 0.1)
            mb.buildMap(['no-trees'])
            output = run + '/' + base + '_bg.npz'
            mb.exportData(output)

    configurator.set('map', True)
