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
from VideoReader import VideoReader
import cv2

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
            self.config[key] = value == 'True'
            self.parser.set('progress', str(key), str(value))
            self.parser.write(config_file)

    def get(self, key, type='bool', default=False):
        if type == 'bool':
            try:
                self.parser.read(self.config_name)
                val = self.parser.get(self.section, key) == 'True'
            except:
                self.set(key, str(default))
                return
        else:
            raise TypeError

        self.config[key] = val

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print """Usage:
        LaneCorrectorPipelineOV.py -f folder/
        -f = force"""
        sys.exit(-1)

    run = sys.argv[-1].split('/')
    if run[-1] == '':
        run = run[-2]
    else:
        run = run[-1]

    local_folder = '/deep/group/driving_data/jkiske/data/' + run
    remote_folder = sys.argv[-1]

    print local_folder, remote_folder
    try:
        os.mkdir(local_folder)
    except OSError:
        pass

    config_file_name = local_folder + '/progress.ini'
    configurator = Config(config_file_name)

    if '-f' in sys.argv:
        configurator.delete()

    configurator.get('downloaded')
    configurator.get('organized')
    configurator.get('map')
    configurator.get('multilane')
    configurator.get('planefitting')
    configurator.get('sync')

    print configurator.config

    if configurator.config['downloaded'] == False:
        cmd = """rsync --progress -a -L --prune-empty-dirs --exclude="*_frames/" \
        --exclude="*_radar" --include="*_frames.tar.gz" \
        --include="*604.zip" --include="*.out" --include="params.ini" \
        --include="*lanes.pickle" --include="*.jpg" --include="*.*proto" \
        --filter="-! */" /deep/group/driving_data/q50_data/{remote} {local}"""\
            .format(remote=remote_folder, local=local_folder + '/..')
        print cmd
        tokens = shlex.split(cmd)
        if '-d' in sys.argv:
            tokens.insert(1, '--dry-run')
            print 'DRY RUN'

        subprocess.call(tokens)

        if not '-d' in sys.argv:
            configurator.set('downloaded', True)

    if configurator.config['organized'] == False:
        for video in glob.glob(local_folder + '/*604.zip'):
            organized_folder = video.replace('604.zip', '')
            print video, organized_folder

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

                    if '.tar.gz' in new_file:
                        untar_cmd = 'tar xf %s -C %s' % (new_file,
                                                         organized_folder)
                        subprocess.call(untar_cmd.split())
                        os.remove(new_file)

            params = local_folder + '/params.ini'
            folder_params = local_folder + '/' + folder_name + '/params.ini'
            shutil.copy(params, folder_params)

        configurator.set('organized', True)

    if configurator.config['map'] == False:
        for run in sorted(glob.glob(local_folder + '/*/')):
            if os.path.isdir(run):
                print run
                base = run.split('/')[-2]
                temp_vid = base + '604.zip'
                args = parse_args(run, temp_vid)
                mb = MapBuilder(args, 1, 600, 0.5, 0.1)

                output = run + '/' + base + '_bg.npz'
                if not os.path.isfile(output):
                    mb.buildMap(['no-trees'])
                    mb.exportData(output)

                output = run + '/' + base + '_ground.npz'
                if not os.path.isfile(output):
                    mb.buildMap(['no-trees', 'ground'])
                    mb.exportData(output)

        configurator.set('map', True)

    if configurator.config['multilane'] == False:
        for run in sorted(glob.glob(local_folder + '/*/')):
            if os.path.isdir(run):
                print run
                num_lanes_file_name = run + '/num_lanes.ini'

                with open(num_lanes_file_name, 'w') as num_lanes_file:
                    num_lanes_file.write('left = 5\n')
                    num_lanes_file.write('right = 5\n')

                interp = glob.glob(run + '/*interp_lanes.pickle')[0]
                bg = glob.glob(run + '/*_bg.npz')[0]
                cmd = 'python -u OffsetLanes.py {interp} {l} {r} {bg} {folder}'
                cmd = cmd.format(interp=interp, l=5, r=5, bg=bg, folder=run)
                print cmd
                subprocess.call(cmd.split())

        configurator.set('multilane', True)

    if configurator.config['planefitting'] == False:
        for run in sorted(glob.glob(local_folder + '/*/')):
            if os.path.isdir(run):
                print run
                if len(glob.glob(run + '/*_planar.npz')) == 0:
                    video = run.split('/')[-2] + '604.zip'
                    cmd = 'python -u PlaneFitting.py {run} {video}'
                    cmd = cmd.format(run=run, video=video)
                    print cmd
                    subprocess.call(cmd.split())

        configurator.set('planefitting', True)

    if configurator.config['sync'] == False:
        driving_data = '/deep/group/driving_data/'
        # sync_cmd = """rsync --progress -a --exclude=*_frames/
        #         --exclude=*.avi --exclude=*.zip
        #         --exclude=*.pickle --exclude=*~ /scr/data/ \
        #         {driving_data}/jkiske/data""".format(driving_data=driving_data)
        # print sync_cmd
        # subprocess.call(sync_cmd.split())

        for run_path in sorted(glob.glob(local_folder + '/*/')):
            run = run_path.split('/')[-2]
            print 'cleaning', run
            for z in glob.glob(run_path + '/*.zip'):
                os.remove(z)
            shutil.rmtree(run_path + '/' + run + '_frames/', ignore_errors=True)
            video_glob = driving_data + 'q50_data/{remote}/{run}60*.zip'\
                .format(remote=remote_folder, run=run)
            for video in sorted(glob.glob(video_glob)):
                link = driving_data + 'jkiske/data/{remote}/{run}/{video}'\
                    .format(remote=remote_folder, run=run,
                            video=video.split('/')[-1])
                # rm_cmd = 'rm ' + link
                cmd = 'ln -s {video} {link}'.format(video=video, link=link)
                print 'linking', video
                subprocess.call(cmd.split())

        configurator.set('sync', True)
