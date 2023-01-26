#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
from basic_package.srv import addition , additionResponse

def server_cb(req):
    rospy.loginfo("Server is running")
    a = 10
    b = 29
    c = a*a + b*b + req.x

    print("I can do all the gymnastics I want here {}".format(c))
    return additionResponse(req.x + req.y)

if __name__ == "__main__":
    rospy.init_node('add_two_ints_server')
    rospy.logwarn("This is no good")
    s = rospy.Service('add_two_integers', addition, server_cb)
    rospy.spin()