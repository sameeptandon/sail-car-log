import os, sys
import subprocess, multiprocessing

GPS_COMMAND = 'python ../gps/BagToGPSMarkOut.py '

target_dir = ''

def run_command(args):
    (p, target_dir) = args
 
    basename = p.split('.')[0]
    gps_bag = basename + '_gps.bag'
    cmd = GPS_COMMAND + "%s/%s" % (target_dir, gps_bag)
    print cmd
    try:
        import rosbag
        subprocess.call(cmd, shell=True)
    except ImportError, e:
        pass

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print """Usage:
            generate_gps_out.py <folder> 
        """
        exit(-1)

    target_dir = sys.argv[1]
    files = os.listdir(target_dir)
    pcap_files = filter(lambda x: '.pcap' in x,  files)

    pool = multiprocessing.Pool(processes=1)
    pool.map(run_command,
        zip(pcap_files, [target_dir]*len(pcap_files))) 
