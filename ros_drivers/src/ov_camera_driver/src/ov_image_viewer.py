#!/usr/bin/env python
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # subscribed Topic
        self.subscriber = rospy.Subscriber("/cam1/image_raw",
            CompressedImage, self.callback,  queue_size = 100)


    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        
        print 'received image of type: "%s"' % ros_data.format
    
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #print image_np.shape
        """
        cv2.imshow('disp', image_np)
        cv2.waitKey(5)
        """


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
