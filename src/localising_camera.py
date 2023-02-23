#!/usr/bin/env python

import rospy
import numpy
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Transform, TransformStamped
from basic_package.srv import transform , transformResponse

class localisation:
	def __init__(self):
		cam_topic_name = "/camera/accel/sample"
		transforms_topic_name = "/fiducial_transforms"
		self.cam_subscriber = rospy.Subscriber(cam_topic_name, Imu, self.cam_callback)
		self.transform_subscriber = rospy.Subscriber(transforms_topic_name, FiducialTransformArray, self.transform_callback)
		self.broadcasting = rospy.Timer(rospy.Duration(0.1),self.broadcast_callback)
		self.timer = rospy.Timer(rospy.Duration(1),self.timer_callback)

	def cam_callback(self,msg):
		global moved
		global movement_deviation
		# reads the magnitude of the linear acceleration vector of the camera and decides whether it is moving
		move_c =[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
		moved = numpy.linalg.norm(move_c)
		movement_deviation.append(moved)

	def transform_callback(self,msg):
		global prev_err
		global is_first_time
		global transformations
		global fid_id 
		# Array of all the fiducial transforms
		marker_transforms = msg.transforms
		# To eliminate errors when no markers visible
		if len(marker_transforms) != 0:
			if is_first_time:
				# initialize the first error values
				for index in range(len(marker_transforms)):
					prev_err[marker_transforms[index].fiducial_id] = marker_transforms[index].object_error
					is_first_time = False
			else:
				# Calculate exponential moving avg of errors
				alpha = 0.2 #constant to apply weight to most recent values
				obj_err = [0.0 for marker in range(4)]
				for index in range(len(marker_transforms)):
					# Getting errors
					obj_err[marker_transforms[index].fiducial_id] = marker_transforms[index].object_error
					prev_err[marker_transforms[index].fiducial_id] = alpha * obj_err[marker_transforms[index].fiducial_id] + (1 - alpha) * prev_err[marker_transforms[index].fiducial_id]
				filtered_index =[]
				for i in range(4):
					# Eliminate values that come from markers that are no longer visible
					if obj_err[i] != 0.0:
						filtered_index.append(i)
				# Just an arbitrary large value
				min_value = 330.0
				for i in filtered_index:
					# finding the minimum value of error
					if prev_err[i] < min_value:
						min_value = prev_err[i]
				min_index = prev_err.index(min_value)
				# publish the required values to a global variable
				for index in range(len(marker_transforms)):
					if min_index == marker_transforms[index].fiducial_id:
						fid_id = marker_transforms[index].fiducial_id
						transformations.translation = marker_transforms[index].transform.translation
						transformations.rotation = marker_transforms[index].transform.rotation

		else:
			# could show blindness via global variable
			rospy.logwarn("I am blind")

	def do_transform_client(self, trans, rot):
		global broadcast_tf
		# client to call whenever a new transformation calc is needed for camera
		rospy.wait_for_service('transform_service')
		do_transform = rospy.ServiceProxy('transform_service', transform)
		input = Transform(trans,rot)
		resp1 = do_transform(input)
		broadcast_tf.translation = resp1.vector
		broadcast_tf.rotation = resp1.quaternion
		print(broadcast_tf.translation)
		print(broadcast_tf.rotation)

	def tf_broadcaster(self, broadcast, link, parent_frame, child_frame, translation, rotation):
		
		# Function to broacast tf's
		link.header.stamp = rospy.Time.now()
		link.header.frame_id = parent_frame
		link.child_frame_id = child_frame

		link.transform.translation = translation

		link.transform.rotation = rotation
		broadcast.sendTransform(link)

		return

	def timer_callback(self,event):
		global movement_deviation
		global transformations
		global fid_id
		global another_first_time
		global id
		# calculate deviation to signal camera movement
		std_dev = numpy.std(movement_deviation)
		rospy.loginfo("Timer going")
		if std_dev > 0.2 or another_first_time: #if cam moves and things are visible
			# Service to calculate and renew camera location
			print("Service called")
			self.do_transform_client(transformations.translation, transformations.rotation)
			id = fid_id # to stop marker from switching when stationary
			another_first_time = False
		movement_deviation = []
	
	def broadcast_callback(self,event):
		global broadcast_tf
		global id
		br = tf2_ros.TransformBroadcaster()
		coords = TransformStamped()

		self.tf_broadcaster(br, coords, "ARuco"+str(id), "camera_link", broadcast_tf.translation, broadcast_tf.rotation)

if __name__ == '__main__':
	try:
		# initialize global variables
		move = 0.0
		id = 0
		movement_deviation = []
		is_first_time = True
		another_first_time = True
		prev_err = [330.0 for i in range(4)]
		transformations = Transform()
		broadcast_tf = Transform()
		rospy.init_node('localising_camera')
		sub_node_obj = localisation()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
