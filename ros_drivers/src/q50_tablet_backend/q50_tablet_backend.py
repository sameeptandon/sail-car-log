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

global done_working
done_working = False

ZMQ_DIAGNOSTICS_SEND_PORT = 5000
ZMQ_Q50_TABLET_BACKEND_PORT = 5001

class ImageManager:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/fwd_left/image_color",Image,self.callback)
        self.lastTime = 0
        self.lastImage = None

    def getImage(self):
        return self.lastImage

    def callback(self,data):

        if time.time() - self.lastTime < 0.5:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError, e:
            print e

        cv_image = cv2.pyrDown(cv_image)
        cv_image = cv2.pyrDown(cv_image)
        self.lastImage = cv_image
        self.lastTime = time.time()

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

def main(args):
    global done_working

    rospy.init_node('q50_tablet_backend')
    basename = rospy.get_param('~basename', 'NONAME')
    maxframes = int(rospy.get_param('~maxframes', '9999999999'))
    print basename, maxframes
    zmq_context = zmq.Context()
    diagnostics_pub = zmq_context.socket(zmq.PUB)
    diagnostics_pub.connect("tcp://localhost:"+str(ZMQ_DIAGNOSTICS_SEND_PORT))
    my_commands = zmq_context.socket(zmq.SUB)
    my_commands.bind("tcp://*:"+str(ZMQ_Q50_TABLET_BACKEND_PORT))
    my_commands.setsockopt(zmq.SUBSCRIBE, '') # subscribe to all messages

    import subprocess
    start_drivers_p = subprocess.Popen(getStartDriversCommand(), shell=True)
    time.sleep(2)
    start_collection_p = subprocess.Popen(getStartCollectionCommand(basename), shell=True)
    time.sleep(3)

    total_execution_time = maxframes / 50;
    start_time = time.time()

    ic = ImageManager()
    done_working = False
    quit_via_user_input = False
    while not done_working:
        print 'hi'
        # check if we should terminate. Two cases: user hits stop or time expires
        msg = poll_msg(my_commands)
        if msg == 'TERMINATE':
            done_working = True
            quit_via_user_input = True
        if time.time() - start_time > total_execution_time:
            done_working = True

        if ic.getImage() is not None:
            (success, img_data) = cv2.imencode('.png', ic.getImage())
            if success:
                diagnostics_pub.send('CAM:' + img_data.tostring(), zmq.NOBLOCK)
        time.sleep(0.5)

    stop_collection_p = subprocess.Popen(getStopCollectionCommand(), shell=True)
    time.sleep(2)
    stop_drivers_p = subprocess.Popen(getStopDriversCommand(), shell=True)

    start_collection_p.wait()
    print 'start collect proc fin'
    start_drivers_p.wait()
    print 'start driver proc fin'
    stop_collection_p.wait()
    print 'stop collect proc fin'
    stop_drivers_p.wait()
    print 'stop drivers proc fin'

    if quit_via_user_input:
        return 1
    else:
        return 0

if __name__ == '__main__':
    retval = main(sys.argv)
    print 'received retval = ', retval
    sys.exit(retval)
