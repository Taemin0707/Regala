#!/usr/bin/env python3

import rospy
import numpy as np

from sklearn.linear_model import LinearRegression

import cv2

from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge.core import CvBridge, CvBridgeError

class VideoStitcher(object):
    """
    """

    def __init__(self):
        """
        """
        # Image Size
        self.width = rospy.get_param('~width', 1920)
        self.height = rospy.get_param('~height', 1080)

        # ROS-OpenCV Bridge
        self.bridge = CvBridge()

        # Time Sync Subscriber
        left_image_sub = Subscriber('/left_camera/image_raw', Image)
        center_image_sub = Subscriber('/center_camera/image_raw', Image)
        right_image_sub = Subscriber('/right_camera/image_raw', Image)

        time_sync = ApproximateTimeSynchronizer([left_image_sub, center_image_sub, right_image_sub], queue_size=1, slop=0.1)
        time_sync.registerCallback(self.callback)

        # Stitched Image Publisher
        self.panorama_image_pub = rospy.Publisher('/panorama/image_raw', Image, queue_size=1)

    def callback(self, left, center, right):
        try:
            left_img = self.bridge.imgmsg_to_cv2(left, "bgr8")
            center_img = self.bridge.imgmsg_to_cv2(center, "bgr8")
            right_img = self.bridge.imgmsg_to_cv2(right, "bgr8")

            # pano_img = cv2.hconcat([left_img, center_img, right_img])
            pano_img = cv2.hconcat([right_img, center_img, left_img])

            # result = cv2.warpPerspective(right_img, homography, (output_shape))
            # # result[0:left_img.shape[0], 0:1280] = left_img[0:left_img.shape[0], 0:1280]
            # result[0:left_img.shape[0], 0:int(center_points[0])] = left_img[0:left_img.shape[0], 0:int(center_points[0])]
                        
            # print(center_points)

            self.panorama_image_pub.publish(self.bridge.cv2_to_imgmsg(pano_img, "bgr8"))

        except CvBridgeError as e:
            print("here")
            print(e)
        
if __name__ == '__main__':
    print("Initializing Video Stitcher")
    rospy.init_node('video_stitcher', anonymous=False)
    video_stitcher = VideoStitcher()
    rospy.spin()