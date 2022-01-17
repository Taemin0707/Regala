#!/usr/bin/env python3

import rospy
import numpy as np

from sklearn.linear_model import LinearRegression

import cv2

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge.core import CvBridge, CvBridgeError

class VideoRecorder(object):
    """
    """

    def __init__(self):
        """
        """
        pano_image_sub = rospy.Subscriber('/panorama/image_raw', Image, self.callback)
 
        # ROS-OpenCV Bridge
        self.bridge = CvBridge()        

        file = './output.avi'
        # file = './video.mp4'

        self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        # self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(file, self.fourcc, 30, (5760, 1080))

    def callback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            self.out.write(cv_image)

        except CvBridgeError as e:
            print(e)
        
if __name__ == '__main__':
    rospy.init_node('video_recorder', anonymous=False)
    video_recorder = VideoRecorder()
    rospy.spin()