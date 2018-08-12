#!/usr/bin/env python

import sys
import rospy
from o2as_precision_gripper.srv import *

# def precision_gripper_client(open_outer_gripper,close_outer_gripper,open_inner_gripper,close_inner_gripper):
#     rospy.wait_for_service('precision_gripper')
#     try:
#         precision_gripper = rospy.ServiceProxy('precision_gripper',PrecisionGripperCommand)
#         response1 = precision_gripper(open_outer_gripper,close_outer_gripper,open_inner_gripper,close_inner_gripper)
#         return response1.success
#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

if __name__ == "__main__":
    open_outer_gripper=True
    close_outer_gripper=False
    open_inner_gripper=False
    close_inner_gripper=False
    # rospy.loginfo(str(precision_gripper_client(open_outer_gripper,close_outer_gripper,open_inner_gripper,close_inner_gripper)))

    rospy.wait_for_service('precision_gripper_command')
    print "kaidi"
    try:
        precision_gripper_client = rospy.ServiceProxy('precision_gripper_command',PrecisionGripperCommand)
        response1 = precision_gripper_client(open_outer_gripper,close_outer_gripper,open_inner_gripper,close_inner_gripper)
        rospy.loginfo("response1.success = " + str(response1.success))
        print response1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
