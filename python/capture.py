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
    name = sys.argv[1] + '_' + getNextSuffix(sys.argv[1])
    command = getCaptureCommand(name)
    signal.signal(signal.SIGINT, signal_handler)
    process = subprocess.Popen(command.split())
    while process.poll() is None:
        time.sleep(1)
        

