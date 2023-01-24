#!/usr/bin/env python

import sys
import rospy

from basic_package.srv import addition , additionResponse

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_integers')
    add_two_ints = rospy.ServiceProxy('add_two_integers', addition)
    resp1 = add_two_ints(x, y)
    print(resp1.result)


if __name__ == "__main__":
    x = 3
    y = 1
    add_two_ints_client(x, y)