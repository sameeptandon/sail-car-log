DIAGNOSTICS_PORT = 5000 
CAMERALOGGER_PORT = 5001

import sys, os, time, signal, subprocess
from file_conventions import *
import zmq 

process = None

context = zmq.Context()
logger_socket= context.socket(zmq.PUB) 
logger_socket.connect("tcp://localhost:"+str(CAMERALOGGER_PORT))
diagnostics_socket = context.socket(zmq.SUB)
diagnostics_socket.bind("tcp://*:"+str(DIAGNOSTICS_PORT))
diagnostics_socket.setsockopt(zmq.SUBSCRIBE, '')


def signal_handler(signal, frame):
    global process
    print 'Pressed ctrl-C'
    logger_socket.send("TERMINATE")
    while process.poll() is None:
        time.sleep(0.1)
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
        process_exit_val = None
        while process_exit_val is None:
            try:
              while True: 
                message = diagnostics_socket.recv(zmq.NOBLOCK)
                print message
            except zmq.core.error.ZMQError, e:
              continue

            process_exit_val = process.poll()
            time.sleep(1)

        if process_exit_val == 1:
            sys.exit(0)
        

