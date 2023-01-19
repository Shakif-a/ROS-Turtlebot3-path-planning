#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

x = False

def callback(msg):
	global x
	a= msg.ranges[0]
	if a < 0.5 and a !=0.0:
		x = True
	print(a)



rospy.init_node('move_turtlebot3_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber('/scan', LaserScan, callback)
rate = rospy.Rate(2)
move = Twist()
move.linear.x = 0.15
move.angular.z = 0.0


#Run for 5 seconds
i = 0
while i < 120:
	pub.publish(move)
	i=i+1
	if x == True:
		move.linear.x = 0.0
	rate.sleep()
	

#Stop motion
while not rospy.is_shutdown():
	connections = pub.get_num_connections()
	if connections > 0:
		move.linear.x = 0.0
		move.angular.z = 0.0
		pub.publish(move)
		rospy.loginfo("Cmd Published")
		print(x)
		break
	else:
		rate.sleep()
