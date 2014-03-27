#!/usr/bin/env python
import roslib
roslib.load_manifest('q50_tablet_backend')
import cv2, cv
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

rospy.init_node('video_loader', anonymous=True)
reader = cv2.VideoCapture("/home/smart/sail-car-log/cameralogger/build/3-19-14-tracy/split_3_to_tracy_g1.avi")

image_pub = rospy.Publisher("/camera_fake/image", Image)
bridge = CvBridge()

import time
while True:
    (success, I) = reader.read()
    if not success:
        break
    try:    
        image_pub.publish(bridge.cv2_to_imgmsg(I, "bgr8"))
    except CvBridgeError, e:
        print e
    time.sleep(0.02)



