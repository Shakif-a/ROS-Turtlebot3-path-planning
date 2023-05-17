#!/usr/bin/env python

import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LineFollower:
    def __init__(self, name):
        self.name = name
        self.bridge = CvBridge()
        self.image = None

        # Define the topic names
        colour_image_topic = "/camera/rgb/image_raw"
        depth_image_topic = "/camera/depth/image_raw"

        # Create the individual subscribers
        colour_sub = message_filters.Subscriber(colour_image_topic, Image)
        depth_sub = message_filters.Subscriber(depth_image_topic, Image)

        # Synchronize messages based on timestamps
        self.camera_sync = message_filters.TimeSynchronizer([colour_sub, depth_sub], 10)
        self.camera_sync.registerCallback(self.callback_camera)

    def callback_camera(self, colour_msg, depth_msg):
        rospy.loginfo("[%s] callback_camera()", self.name)
        colour_image = self.bridge.imgmsg_to_cv2(colour_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

        # Find target object based on HSV values
        hsv = cv2.cvtColor(colour_image, cv2.COLOR_BGR2HSV)
        lower_yellow = (25, 200, 100)
        upper_yellow = (35, 255, 255)
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Keep only the lower band of the image
        height, width = mask.shape[:2]
        search_top = int(3 * height / 4)
        search_bot = search_top + 20
        mask[0:search_top, 0:width] = 0
        mask[search_bot:height, 0:width] = 0

        self.image = mask

        cv2.imshow("window", self.image)
        cv2.waitKey(30)

if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)
    rospy.loginfo("[line_follower] Starting Line Follower Module")
    lf = LineFollower("line_follower")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[line_follower] Shutting Down Line Follower Module")