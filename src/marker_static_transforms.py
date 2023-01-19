#!/usr/bin/env python
import rospy

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':

    rospy.init_node('my_static_tf2_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    #Aruco marker 1
    static_transformStamped1 = geometry_msgs.msg.TransformStamped()
    static_transformStamped1.header.stamp = rospy.Time.now()
    static_transformStamped1.header.frame_id = "base_scan"
    static_transformStamped1.child_frame_id = "ARuco1"

    static_transformStamped1.transform.translation.x = -0.0663
    static_transformStamped1.transform.translation.y = 0.08031
    static_transformStamped1.transform.translation.z = -0.0300

    static_transformStamped1.transform.rotation.x = 0.0
    static_transformStamped1.transform.rotation.y = 0.0
    static_transformStamped1.transform.rotation.z = -0.7071068
    static_transformStamped1.transform.rotation.w = 0.7071068



    #Aruco marker 0
    static_transformStamped0 = geometry_msgs.msg.TransformStamped()
    static_transformStamped0.header.stamp = rospy.Time.now()
    static_transformStamped0.header.frame_id = "base_scan"
    static_transformStamped0.child_frame_id = "ARuco0"

    static_transformStamped0.transform.translation.x = 0.09371
    static_transformStamped0.transform.translation.y = 0.08031
    static_transformStamped0.transform.translation.z = -0.0300

    static_transformStamped0.transform.rotation.x = 0.0
    static_transformStamped0.transform.rotation.y = 0.0
    static_transformStamped0.transform.rotation.z = -0.7071068
    static_transformStamped0.transform.rotation.w = 0.7071068



    #Aruco marker 2
    static_transformStamped2 = geometry_msgs.msg.TransformStamped()
    static_transformStamped2.header.stamp = rospy.Time.now()
    static_transformStamped2.header.frame_id = "base_scan"
    static_transformStamped2.child_frame_id = "ARuco2"

    static_transformStamped2.transform.translation.x = 0.09351
    static_transformStamped2.transform.translation.y = -0.0798
    static_transformStamped2.transform.translation.z = -0.0300

    static_transformStamped2.transform.rotation.x = 0.0
    static_transformStamped2.transform.rotation.y = 0.0
    static_transformStamped2.transform.rotation.z = -0.7071068
    static_transformStamped2.transform.rotation.w = 0.7071068



    #Aruco marker 3
    static_transformStamped3 = geometry_msgs.msg.TransformStamped()
    static_transformStamped3.header.stamp = rospy.Time.now()
    static_transformStamped3.header.frame_id = "base_scan"
    static_transformStamped3.child_frame_id = "ARuco3"

    static_transformStamped3.transform.translation.x = -0.0665
    static_transformStamped3.transform.translation.y = -0.0798
    static_transformStamped3.transform.translation.z = -0.0300

    static_transformStamped3.transform.rotation.x = 0.0
    static_transformStamped3.transform.rotation.y = 0.0
    static_transformStamped3.transform.rotation.z = -0.7071068
    static_transformStamped3.transform.rotation.w = 0.7071068

    broadcaster.sendTransform(static_transformStamped0)
    rospy.sleep(0.5)
    broadcaster.sendTransform(static_transformStamped1)
    rospy.sleep(0.5)
    broadcaster.sendTransform(static_transformStamped2)
    rospy.sleep(0.5)
    broadcaster.sendTransform(static_transformStamped3)
    rospy.sleep(0.5)
    rospy.spin()