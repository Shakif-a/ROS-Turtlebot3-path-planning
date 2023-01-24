#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion

class badboy:

    def __init__(self):
        self.bully = 0.0
        self.broadcaster = tf2_ros.TransformBroadcaster()
        rospy.logwarn("This is no good")
    def loop(self):
        translation = Vector3(-1.1,self.bully,.0)
        rotation = Quaternion(.0,.0,.0, 1.0)
        trans = Transform(translation,rotation)
        static_marka = TransformStamped()
        static_marka.header.stamp = rospy.Time.now()
        static_marka.header.frame_id = "map"
        static_marka.child_frame_id = "camera_link"
        static_marka.transform = trans
        self.broadcaster.sendTransform(static_marka)
        rospy.loginfo("Running")
        self.bully += 0.1

if __name__ == '__main__':
    rospy.init_node("Badboy_node")
    r = badboy()
    time = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        time_d = now - time
        if time_d.secs < 5 or time_d.secs > 20 and time_d.secs < 30:
            r.loop()
            rospy.sleep(1)
        