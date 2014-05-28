import os, sys
import subprocess, multiprocessing

FF_COMMAND = 'python FrameFinder.py '
GPS_COMMAND = 'python ../gps/BagToGPSOut.py '

params = ''
target_dir = ''

def run_command(args):
    (p, target_dir) = args
    
    param_file = target_dir + '/params.ini'
    f = open(param_file, 'w')
    f.write(params + '\n')
    f.close()
    print 'Wrote %s to %s' % (params, param_file)
 
    basename = p.split('.')[0]
    gps_bag = basename + '_gps.bag'
    cmd = GPS_COMMAND + "%s/%s" % (target_dir, gps_bag)
    print cmd
    try:
        import rosbag
        subprocess.call(cmd, shell=True)
    except ImportError, e:
        pass


    gps_file = basename + '_gps.out'
    frames_folder = basename + "_frames"
    rdr_bag = basename + "_radar.bag"
    cmd = FF_COMMAND + "{t}/{gps} {t}/{frames} {t}/{rdr}".format(
        t=target_dir, gps=gps_file, frames=frames_folder, rdr=rdr_bag)
    print cmd
    subprocess.call(cmd, shell=True)


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print """Usage:
            generate_frames_and_map.py <folder> <which parameter file to load>"""
        exit(-1)

    target_dir = sys.argv[1]
    files = os.listdir(target_dir)
    pcap_files = filter(lambda x: '.pcap' in x,  files)

    params = sys.argv[2]
    pool = multiprocessing.Pool(processes=1)
    pool.map(run_command,
        zip(pcap_files, [target_dir]*len(pcap_files))) 
