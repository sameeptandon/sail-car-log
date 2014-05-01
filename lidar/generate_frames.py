import os, sys
import subprocess, multiprocessing

LDR_COMMAND = 'build/LDRConverter --hdlcalibration 32db.xml --p '

params = ''
target_dir = ''

def run_command(args):
    (p, target_dir) = args
    
    param_file = target_dir + '/params.ini'
    f = open(param_file, 'w')
    f.write(params + '\n')
    f.close()
    print 'Wrote %s to %s' % (params, param_file)

    cmd = LDR_COMMAND + target_dir + '/' + p
    print cmd
    subprocess.call(cmd, shell=True)

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print """Usage:
            generate_frames.py <folder> <which parameter file to load>"""
        exit(-1)

    target_dir = sys.argv[1]
    files = os.listdir(target_dir)
    pcap_files = filter(lambda x: '.pcap' in x,  files)

    params = sys.argv[2]
    pool = multiprocessing.Pool(processes=1)
    pool.map(run_command,
        zip(pcap_files, [target_dir]*len(pcap_files))) 
