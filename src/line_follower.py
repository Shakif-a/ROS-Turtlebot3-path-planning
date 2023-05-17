#!/usr/bin/env python

import rospy, message_filters, cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class LineFollower:
    def __init__(self, name):
        self.name = name

if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)
    rospy.loginfo("[line_follower] Starting Line Follower Module")
    lf = LineFollower("line_follower")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("[line_follower] Shutting Down Line Follower Module")
