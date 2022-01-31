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
        rospy.Subscriber('/panorama/image_raw', Image, self.callback)
 
        # ROS-OpenCV Bridge
        self.bridge = CvBridge()        

        self.cv_image = np.zeros((720, 3840, 3), dtype=np.uint8)

        file = './output.avi'
        # file = './video.mp4'

        self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        # self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(file, self.fourcc, 20, (3840, 720))

        self.time_start = rospy.Time.now()
        rate = rospy.Rate(20) # 30hz
        frame = 1
        while not rospy.is_shutdown():
            self.out.write(self.cv_image)
            # print((rospy.Time.now() - self.time_start).to_sec())
            rate.sleep()
            # print(frame)

            frame += 1
            if (rospy.Time.now() - self.time_start).to_sec() > 60:
                print("Done")
                break
    def callback(self, image):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")

        except CvBridgeError as e:
            print(e)
        
if __name__ == '__main__':
    rospy.init_node('video_recorder', anonymous=False)
    video_recorder = VideoRecorder()
    rospy.spin()