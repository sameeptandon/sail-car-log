import os, sys
import subprocess, multiprocessing

LDR_COMMAND = 'build/LDRConverter --hdlcalibration 32db.xml --p '
FF_COMMAND = 'python FrameFinder.py '

def run_command(args):
    (p, target_dir) = args
    cmd = LDR_COMMAND + target_dir + '/' + p
    print cmd
    subprocess.call(cmd, shell=True)
    
    basename = p.split('.')[0]
    gps_file = basename + '_gps.out'
    frames_folder = basename + "_frames"
    cmd = FF_COMMAND + "%s/%s %s/%s" % (target_dir, gps_file, target_dir, frames_folder)
    print cmd
    subprocess.call(cmd, shell=True)

target_dir = sys.argv[1]
files = os.listdir(target_dir)
pcap_files = filter(lambda x: '.pcap' in x,  files)

pool = multiprocessing.Pool(processes=6)
pool.map(run_command, 
    zip(pcap_files, 
      [target_dir]*len(pcap_files))) 

