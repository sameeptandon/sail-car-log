import sys, os, time, signal, subprocess
from file_conventions import *

process = None

def signal_handler(signal, frame):
    global process
    print 'Pressed ctrl-C'
    process.send_signal(signal)
    while process.poll() is None:
        time.sleep(1)
    sys.exit(0)

if __name__ == '__main__':
    maxFrames = None
    if len(sys.argv) > 2: 
        maxFrames = int(sys.argv[2])
    signal.signal(signal.SIGINT, signal_handler)
    while True:
        name = sys.argv[1] + '_' + getNextSuffix(sys.argv[1])
        command = getCaptureCommand(name, maxFrames=maxFrames)
        #launch 
        process = subprocess.Popen(command.split())
        #wait to end
        while process.poll() is None:
            time.sleep(1)
        

