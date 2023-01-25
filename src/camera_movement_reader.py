#!/usr/bin/env python

import rospy
import numpy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Vector3, Transform, TransformStamped
from fiducial_msgs.msg import FiducialTransformArray
from sensor_msgs.msg import Imu
from basic_package.srv import addition , additionResponse

class camera_movement:

	def __init__(self):
		#initialise variables
		# topic to see if camera moves from camera_acceleration
		sub2_topic_name ="/camera/accel/sample"
		self.cam_subscriber = rospy.Subscriber(sub2_topic_name, Imu, self.cam_callback)
	
	def cam_callback(self, msg):
		global move
		global movement_dev
		# reads the magnitude of the linear acceleration vector of the camera and decides whether it is moving
		move_c =[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
		# this is roughly the value of it when stationary 
		move = numpy.linalg.norm(move_c)
		movement_dev.append(move)
		
	def timer_callback(self, event):
		global movement_dev
		global move
		global integer
		global counter

		std_dev = numpy.std(movement_dev)
		rospy.loginfo("Timer going")

		if std_dev > 0.2:
			print("CONTACT!!")
			self.add_two_ints_client(std_dev, 4)
		movement_dev = []

		if integer != 0 and counter <= 25:
			static_transform = TransformStamped()
			static_transform.header.stamp = rospy.Time.now()
			static_transform.header.frame_id = "map"
			static_transform.child_frame_id = "camera_link"
			static_transform.transform.translation = Vector3(0.2 + integer, 1.2, 0.0)
			static_transform.transform.rotation = Quaternion(0.0,0.0,0.0,1.0)
			broadcaster.sendTransform(static_transform)
			counter += 1
			print(counter)

	def add_two_ints_client(self, x, y):
		rospy.wait_for_service('add_two_integers')
		add_two_ints = rospy.ServiceProxy('add_two_integers', addition)
		resp1 = add_two_ints(x, y)
		print(resp1.result)
		global integer
		integer = resp1.result

if __name__ == '__main__':
	node_name ="move_camera"
	rospy.init_node(node_name)
	move = 0.0
	counter = 1
	movement_dev = []
	broadcaster = tf2_ros.TransformBroadcaster()
	integer = 0
	r = camera_movement()
	timer = rospy.Timer(rospy.Duration(1),r.timer_callback)
	rospy.spin()