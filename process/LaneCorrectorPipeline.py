#!/usr/bin/python

from ConfigParser import SafeConfigParser
import os
import subprocess
import sys
import shlex
import os.path


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
    configurator.get('mapped')
else:
    configurator.set('downloaded', False)
    configurator.set('mapped', False)

print configurator.config

if configurator.config['downloaded'] == False:
    cmd = """rsync --progress -a --prune-empty-dirs --include="*.rdr" \
    --include="*.ldr" --include="*.avi" --include="*.out" \
    --include="params.ini" --include="*lanes.pickle" --filter="-! */" \
    jkiske@gorgon33:~/q50_data/{remote} {local}""".format(
        remote=remote_folder, local=local_folder)

    tokens = shlex.split(cmd)
    if '-d' in sys.argv:
        tokens.insert(1, '--dry-run')

    print tokens
    subprocess.call(tokens)

    configurator.set('downloaded', True)
