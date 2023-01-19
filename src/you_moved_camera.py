#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu


class cam_movement:
	i = False
	def __init__(self):
		
		sub2_topic_name ="/camera/accel/sample"
		self.cam_subscriber = rospy.Subscriber(sub2_topic_name, Imu, self.cam_callback)
		rospy.spin()

	def cam_callback(self, msg):
		move_c =[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z] 
		base_value = 9.677
		move = numpy.linalg.norm(move_c)
		if move > base_value + 0.6:
			self.i = True
			print("You moved "+str(self.i))
		elif move < base_value - 0.6:
			self.i = True
			print("You moved "+str(self.i))
		else:
			self.i = False



if __name__ == '__main__':
	try:
		rospy.init_node('you_moved_camera')
		sn = cam_movement()
	except rospy.ROSInterruptException:
		pass
