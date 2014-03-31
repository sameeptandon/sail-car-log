import os, sys
import subprocess, multiprocessing

LDR_COMMAND = 'build/LDRConverter --hdlcalibration 32db.xml --p '
FF_COMMAND = 'python FrameFinder.py '
GPS_COMMAND = 'python ../gps/BagToGPSOut.py '

def run_command(args):
    (p, target_dir) = args
    cmd = LDR_COMMAND + target_dir + '/' + p
    print cmd
    subprocess.call(cmd, shell=True)
 
    basename = p.split('.')[0]
    gps_bag = basename + '_gps.bag'
    cmd = GPS_COMMAND + "%s/%s" % (target_dir, gps_bag)
    print cmd
    subprocess.call(cmd, shell=True)


    gps_file = basename + '_gps.out'
    frames_folder = basename + "_frames"
    rdr_bag = basename + "_radar.bag"
    cmd = FF_COMMAND + "{t}/{gps} {t}/{frames} {t}/{rdr}".format(
        t=target_dir, gps=gps_file, frames=frames_folder, rdr=rdr_bag)
    print cmd
    subprocess.call(cmd, shell=True)

target_dir = sys.argv[1]
files = os.listdir(target_dir)
pcap_files = filter(lambda x: '.pcap' in x,  files)

pool = multiprocessing.Pool(processes=1)
pool.map(run_command, 
    zip(pcap_files, 
      [target_dir]*len(pcap_files))) 

