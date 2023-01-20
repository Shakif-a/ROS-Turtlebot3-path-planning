#!/usr/bin/env python

# Just to have fun with quaternions, visualize them for my own understanding of how they work
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import Quaternion, Vector3, Transform, TransformStamped
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from tf.transformations import *

def vector3_add(vector1, vector2):
    vector_r = Vector3()
    vector_r.x=vector1.x + vector2.x
    vector_r.y=vector1.y + vector2.y
    vector_r.z=vector1.z + vector2.z

    return vector_r

def quat_to_array(quaternion):
    array = [0,0,0,0]
    array[0] = quaternion.x
    array[1] = quaternion.y
    array[2] = quaternion.z
    array[3] = quaternion.w

    return array

def array_to_quat(array):
    quaternion = Quaternion()

    quaternion.x = array[0]
    quaternion.y = array[1]
    quaternion.z = array[2] 
    quaternion.w = array[3]

    return quaternion

def reverse_translate(translation):
    result = Vector3()
    result.x = -translation.x
    result.y = -translation.y
    result.z = -translation.z

    return result

def tf_broadcaster(broadcast, link, parent_frame, child_frame, translation, rotation):

    link.header.stamp = rospy.Time.now()
    link.header.frame_id = parent_frame
    link.child_frame_id = child_frame

    link.transform.translation = translation

    link.transform.rotation = rotation
    broadcast.sendTransform(link)

    return

def transform_callback(msg):
    global t 

    global image_errors
    global object_errors

    t = msg.transforms
    image_error = t[0].image_error
    object_error = t[0].object_error
    image_errors.append(image_error)
    object_errors.append(object_error)

def callback(event):
    global image_errors
    global object_errors

    mean = np.mean(image_errors)
    std_dev = np.std(image_errors)

    mean2 = np.mean(object_errors)
    std_dev2 = np.std(object_errors)

    print("Mean is {:.2f}".format(mean))
    print("Std dev is {:.2f} \n".format(std_dev))

    print("O Mean is {:.2f}".format(mean2))
    print("O Std dev is {:.2f}".format(std_dev2))

    rospy.signal_shutdown("Shutting down ......")

rospy.init_node('static_tf')
t = [FiducialTransform() for i in range(2)]
#check which is better to use - image or object error or fid area
image_errors = []
object_errors = []
starting_time = rospy.Time.now()
difference = rospy.Duration(1)

sub_topic_name ="/fiducial_transforms"
transform_subscriber = rospy.Subscriber(sub_topic_name, FiducialTransformArray, transform_callback)

timer = rospy.Timer(rospy.Duration(10),callback)

# rate = rospy.Rate(1)
# while not rospy.is_shutdown():
#     print(t[0].image_error)
#     rate.sleep()

quat_msg = Quaternion(0.707,.0,0.0,0.707)
trans_msg = Vector3(1.0, 0.0, 0.0)

quat_init = Quaternion(0.4963, 0.4963, 0.0, -0.71236)
trans_init = Vector3(0.8, 0.0, 0.5)


broadcast = tf2_ros.StaticTransformBroadcaster()
base_link = TransformStamped()

tf_broadcaster(broadcast, base_link, "robot1", "camera_link", Vector3(-0.1,0.0,.0),Quaternion(.0,.0,.0,1.0))
rospy.sleep(1)
tf_broadcaster(broadcast, base_link, "robot1", "turtle", trans_init, quat_init)


bigbroadcast = tf2_ros.StaticTransformBroadcaster()
next_link = TransformStamped()

tf_broadcaster(bigbroadcast, next_link, "robot1", "turtle1", trans_msg, quat_msg)
rospy.sleep(1)

quat_init_ar = quat_to_array(quat_init)
quat_msg_ar = quat_to_array(quat_msg)

quat_init_ar[3] = -quat_init_ar[3]

f = array_to_quat(quat_init_ar)


new_lin = TransformStamped()
t1 = reverse_translate(trans_init)
t2 = trans_msg
t0 = Vector3(0,0,0)
r0 = Quaternion(0,0,0,1)

tf_broadcaster(broadcast, new_lin, "turtle", "b1", t0, f)
tf_broadcaster(broadcast, new_lin, "b1", "b2", t1, r0)
tf_broadcaster(broadcast, new_lin, "b2", "replacement", trans_msg, quat_msg)
rospy.sleep(1)
rospy.spin()
#another = TransformStamped()
#tf_broadcaster(broadcast, new_lin, "back", "backagain", t1, n)