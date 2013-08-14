import os, sys

def getNextSuffix(prefix):
    files = os.listdir(os.getcwd())
    gps_logs = filter(lambda x: '.out' in x and prefix in x, files)
    return chr(ord('a') + len(gps_logs))

def getCaptureCommand(name):
    cmd = 'sudo /home/smart/sail-car-log/cameralogger/build/CameraLogger -s /dev/serial/by-id/usb-09d7_0210-if00 -o ' + name
    return cmd
