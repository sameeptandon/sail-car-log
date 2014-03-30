#!/usr/bin/env python
import roslib
roslib.load_manifest('q50_tablet_backend')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import zmq
from math import sqrt

global done_working
done_working = False

ZMQ_DIAGNOSTICS_SEND_PORT = 5000
ZMQ_Q50_TABLET_BACKEND_PORT = 5001

class RosTopicManager:
    def __init__(self):
        self.bridge = CvBridge()
        self.lastImageTime = 0
        self.lastImage = None
        self.writer_ack_left_counter = 0
        self.writer_ack_right_counter = 0
        self.nextLineInsCov = False
        self.gps_markpvaa_counter = 0 
        (self.tx, self.ty, self.tz, self.rx, self.ry, self.rz) = (0,0,0,0,0,0)
        (self.lat, self.lon) = (0,0)
        self.image_sub = rospy.Subscriber("/fwd_left/image_raw",Image,self.image_callback)
        self.writer_sub_left = rospy.Subscriber("/fwd_left_writer/writer_ack",String,self.writer_ack_left_callback)
        self.writer_sub_right = rospy.Subscriber("/fwd_right_writer/writer_ack",String,self.writer_ack_right_callback)
        self.gps_sub = rospy.Subscriber("/novatel_port_out",String,self.gps_callback)

    def getImage(self):
        return self.lastImage

    def getWriterAckCount(self):
        return (self.writer_ack_right_counter, self.writer_ack_left_counter, self.gps_markpvaa_counter)

    def getLatLong(self):
        return '%.4f,%.4f' % (self.lat, self.lon)

    def getGPSUncertainty(self):
        tokens = (self.tx, self.ty, self.tz, self.rx, self.ry, self.rz)
        return '%.2f,%.2f,%.2f\n%.2f,%.2f,%.2f' % tokens

    def writer_ack_left_callback(self, data):
        self.writer_ack_left_counter += 1

    def writer_ack_right_callback(self, data):
        self.writer_ack_right_counter += 1

    def gps_callback(self, msg):
        header = msg.data.split(',')[0]

        if 'MARK1PVAA' in header:
            self.gps_markpvaa_counter += 1
            header_tokens = msg.data.split(';')
            gps_data = header_tokens[1]
            tokens = gps_data.split(',')
            self.lat = float(tokens[2])
            self.lon = float(tokens[3])
        if 'INSCOV' in header:
            self.nextLineInsCov = True
            return
        if self.nextLineInsCov:
            self.nextLineInsCov = False 
            tokens = msg.data.split(' ')
            offset = 7
            self.tx = sqrt(float(tokens[offset + 0]))
            self.ty = sqrt(float(tokens[offset + 4]))
            self.tz = sqrt(float(tokens[offset + 8]))
            offset += 9
            self.rx = sqrt(float(tokens[offset + 0]))
            self.ry = sqrt(float(tokens[offset + 4]))
            self.rz = sqrt(float(tokens[offset + 8]))



    def image_callback(self,data):

        if time.time() - self.lastImageTime < 0.5:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError, e:
            print e

        cv_image = cv2.pyrDown(cv_image)
        cv_image = cv2.pyrDown(cv_image)
        self.lastImage = cv_image
        self.lastImageTime = time.time()

def poll_msg(sock):
    try:
        msg = sock.recv(zmq.NOBLOCK)
    except zmq.ZMQError, e:
        if e.errno == zmq.EAGAIN:
            pass
        else:
            raise
    else:
        return msg

def getStartDriversCommand():
    return 'rosrun q50_launch start_drivers.sh'

def getStopDriversCommand():
    return 'rosrun q50_launch stop_drivers.sh'

def getStartCollectionCommand(basename):
    return 'rosrun q50_launch start_data_collection.sh ' + basename;

def getStopCollectionCommand():
    return 'rosrun q50_launch stop_data_collection.sh' 

def sendDiagnosticsMessage(pub_sock, msg):
    if msg[0] != 'C':
        print msg
    pub_sock.send(msg, zmq.NOBLOCK)

def main(args):
    global done_working

    rospy.init_node('q50_tablet_backend')
    basename = rospy.get_param('~basename', 'NONAME')
    maxframes = int(rospy.get_param('~maxframes', '9999999999'))
    print basename, maxframes
    zmq_context = zmq.Context()
    diagnostics_pub = zmq_context.socket(zmq.PUB)
    diagnostics_pub.connect("tcp://localhost:"+str(ZMQ_DIAGNOSTICS_SEND_PORT))
    sendMessage = lambda x: sendDiagnosticsMessage(diagnostics_pub, x)
    my_commands = zmq_context.socket(zmq.SUB)
    my_commands.bind("tcp://*:"+str(ZMQ_Q50_TABLET_BACKEND_PORT))
    my_commands.setsockopt(zmq.SUBSCRIBE, '') # subscribe to all messages

    rt = RosTopicManager()
    sendMessage('WARN:Starting Drivers')

    import subprocess
    start_drivers_p = subprocess.Popen(getStartDriversCommand(), shell=True)
    time.sleep(2)
    sendMessage('WARN:Starting Recorders')
    start_collection_p = subprocess.Popen(getStartCollectionCommand(basename), shell=True)
    time.sleep(3)

    total_execution_time = maxframes / 50;
    start_time = time.time()

    done_working = False
    quit_via_user_input = False
    while not done_working:
        # check if we should terminate. Two cases: user hits stop or time expires
        msg = poll_msg(my_commands)
        if msg == 'TERMINATE':
            done_working = True
            quit_via_user_input = True
        if time.time() - start_time > total_execution_time:
            done_working = True
        
        if rt.getImage() is not None:
            (success, img_data) = cv2.imencode('.png', rt.getImage())
            if success:
                sendMessage('CAM:' + img_data.tostring())

        sendMessage('INFOCAPTURERATE:' + str(rt.getWriterAckCount()))
        sendMessage('GPS:' + str(rt.getLatLong()))
        sendMessage('GPSUNCERTAINTY:'+ str(rt.getGPSUncertainty()))
        time.sleep(1.0)

    sendMessage('WARN:Stopping Recorders')
    stop_collection_p = subprocess.Popen(getStopCollectionCommand(), shell=True)
    time.sleep(6)
    sendMessage('WARN:Stopping Drivers')
    stop_drivers_p = subprocess.Popen(getStopDriversCommand(), shell=True)

    start_collection_p.wait()
    print 'start collect proc fin'
    start_drivers_p.wait()
    print 'start driver proc fin'
    stop_collection_p.wait()
    print 'stop collect proc fin'
    stop_drivers_p.wait()
    print 'stop drivers proc fin'
    
    sendMessage('INFOCAPTURERATE:' + str(rt.getWriterAckCount()))

    if quit_via_user_input:
        return 1
    else:
        return 0

if __name__ == '__main__':
    retval = main(sys.argv)
    print 'received retval = ', retval
    sys.exit(retval)
