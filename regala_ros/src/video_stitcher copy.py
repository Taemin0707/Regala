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
        height = rospy.get_param('~height', 1080)
        width = rospy.get_param('~width', 1920)

        # ROS-OpenCV Bridge
        self.bridge = CvBridge()

        # SURF
        self.sift = cv2.SIFT_create()

        # Time Sync Subscriber
        left_image_sub = Subscriber('/left_camera/image_raw', Image)
        right_image_sub = Subscriber('/right_camera/image_raw', Image)

        # left_image_sub = Subscriber('/calibrated_left_image_raw', Image)
        # right_image_sub = Subscriber('/calibrated_right_image_raw', Image)
        
        time_sync = ApproximateTimeSynchronizer([left_image_sub, right_image_sub], queue_size=10, slop=0.5)
        time_sync.registerCallback(self.callback)

        # Stitched Image Publisher
        self.stitched_image_pub = rospy.Publisher('/stitched_image_raw', Image, queue_size=10)

    def callback(self, left, right):
        try:
            left_img = self.bridge.imgmsg_to_cv2(left, "bgr8")
            right_img = self.bridge.imgmsg_to_cv2(right, "bgr8")

            # Gray Images for detecting features
            gray_left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
            gray_right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

            l_keypoints, l_desc = self.sift.detectAndCompute(gray_left_img, None)
            r_keypoints, r_desc = self.sift.detectAndCompute(gray_right_img, None)

            # FLANN parameters
            FLANN_INDEX_KDTREE = 0
            index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
            search_params = dict(checks=50)

            flann = cv2.FlannBasedMatcher(index_params, search_params)

            # Matching descriptors
            matches = flann.knnMatch(l_desc, r_desc, k=2)

            good_matches = []

            for i, (m, n) in enumerate(matches): 
                if m.distance < 0.40 * n.distance: 
                    good_matches.append(m)
                    
            l_points = []
            r_points = []

            for i in range(len(good_matches)):
                l_points.append(l_keypoints[good_matches[i].queryIdx].pt)
                r_points.append(r_keypoints[good_matches[i].trainIdx].pt)

            l_points = np.array(l_points)
            r_points = np.array(r_points)
            t_r_point = r_points + 1920

            center_points = np.concatenate((l_points, t_r_point), axis=0)
            center_points = np.mean(center_points, axis=0)

            homography, _ = cv2.findHomography(r_points, l_points)

            output_shape = (left_img.shape[1] + right_img.shape[1], left_img.shape[0])
            result = cv2.warpPerspective(right_img, homography, (output_shape))
            # result[0:left_img.shape[0], 0:1280] = left_img[0:left_img.shape[0], 0:1280]
            result[0:left_img.shape[0], 0:int(center_points[0])] = left_img[0:left_img.shape[0], 0:int(center_points[0])]
                        
            print(center_points)

            self.stitched_image_pub.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))

        except CvBridgeError as e:
            print("here")
            print(e)
        
if __name__ == '__main__':
    print("Initializing Video Stitcher")
    rospy.init_node('video_stitcher', anonymous=False)
    video_stitcher = VideoStitcher()
    rospy.spin()