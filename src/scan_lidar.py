#!/usr/bin/env python

#read from /scan topic

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
	a= msg.ranges[0]
	print(a)

rospy.init_node('read_lidar')

sub = rospy.Subscriber('/scan', LaserScan, callback)

rospy.spin()
