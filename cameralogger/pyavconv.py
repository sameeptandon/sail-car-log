import os
import sys
import subprocess
from subprocess import Popen
from time import sleep


def runCommand(command):
    print command
    proc = Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = proc.communicate()
    print out, err 

def compressFile(filename, rate, bitrate=20000000):
    name = extractName(filename)
    num = extractNum(filename)
    command = "avconv -i %s -vcodec mpeg4 -pix_fmt yuv420p -b:v %d -r %d comp-%s-%s.mp4" % (filename,
            bitrate, rate, name, num)
    runCommand(command)

def extractName(f):
    return f.split('-')[0]

def extractNum(f):
    n = f.split('-')[1]
    return n.split('.')[0]

def removeFile(filename):
    command = 'rm -f %s' % filename
    runCommand(command)


def main(fname, rate):
    while True:
        files = os.listdir('.')
        possible = filter(lambda z: fname in z and 'avi' in z, files)
        str_nums = map(lambda x: extractNum(x), possible)
        int_nums = map(lambda x: int(x), str_nums)

        fidx = [ ] 
        for i in range(len(possible)):
            if int_nums[i] + 1 in int_nums:
                fidx.append(i)

        map(lambda x: compressFile(possible[x], rate), fidx)
        map(lambda x: removeFile(possible[x]), fidx)
        sleep(1)


if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]))



