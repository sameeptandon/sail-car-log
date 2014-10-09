#!/usr/bin/env python
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2, cv

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError



class image_converter: 

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish

        self.bridge = CvBridge()

        out_image = rospy.get_param(rospy.search_param('out'))
        image = rospy.get_param(rospy.search_param('image'))
        # subscribed Topic
        self.image_pub = rospy.Publisher(out_image, Image)
        self.subscriber = rospy.Subscriber(image, CompressedImage, self.callback, queue_size = 1000)


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_cv = cv.fromarray(image_np)

        # Publish new image
        try: 
            self.image_pub.publish(self.bridge.cv_to_imgmsg(image_cv, "bgr8"))
        except CvBridgeError, e:
            print e
        

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
