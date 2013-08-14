import sys, os, time, signal, subprocess
from file_conventions import *

process = None

if __name__ == '__main__':
    previousSuffix = None
    while True:
        currentSuffix = chr(ord(getNextSuffix(sys.argv[1])) - 1)
        if currentSuffix != previousSuffix:
            if process:
                process.terminate()
            
            name = sys.argv[1] + '_' + currentSuffix + '_gps.out'
            command = 'tail -f ' + name
            process = subprocess.Popen(command.split(), stdin = subprocess.PIPE)
            previousSuffix = currentSuffix
        time.sleep(5)
