#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_turtlebot3_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.1
move.angular.z = 0.2

#Run for 5 seconds
i=0
while i < 20:
	pub.publish(move)
	i=i+1
	rate.sleep()

#Stop motion
while not rospy.is_shutdown():
	connections = pub.get_num_connections()
	if connections > 0:
		move.linear.x = 0.0
		move.angular.z = 0.0
		pub.publish(move)
		rospy.loginfo("Cmd Published")
		break
	else:
		rate.sleep()
