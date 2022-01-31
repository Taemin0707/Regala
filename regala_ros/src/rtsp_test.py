#!/usr/bin/env python3

import os
import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge.core import CvBridge, CvBridgeError

def camera_node(camera, image_size):
    camera_info = {"right" : 4002, "center" : 4001, "left" : 4000}
    # camera_info = {"right" : 92, "center" : 91, "left" : 90}
    url_num = camera_info[str(camera)]        
    rtsp_url = "rtsp://taemin:kist@211.114.183.94:" + str(url_num) + "/axis-media/media.amp"
    # rtsp_url = "rtsp://taemin:kist@192.168.0." + str(url_num) + ":554/axis-media/media.amp"
    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)

    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"

    topic_name = node_name + "/image_raw"
    image_pub = rospy.Publisher(topic_name, Image, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        # if frame is read correctly ret is True
        frame = cv2.resize(frame, dsize=image_size, interpolation=cv2.INTER_AREA)
        image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node("camera", anonymous=False)
    source = rospy.get_param('~source', "right")
    image_width = rospy.get_param('~image_width', 1920)
    image_height = rospy.get_param('~image_height', 1080)
    
    image_size = (image_width, image_height)

    node_name = str(source) + "_camera"
    print("Initializing " + node_name)

    try:
        camera_node(source, image_size)
    except rospy.ROSInterruptException:
        pass