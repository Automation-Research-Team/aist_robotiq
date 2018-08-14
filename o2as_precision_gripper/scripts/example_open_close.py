#!/usr/bin/env python

import sys
import rospy
from o2as_msgs.srv import *
# from o2as_precision_gripper.srv import *

if __name__ == "__main__":
    rospy.wait_for_service('precision_gripper_command')
    try:
        precision_gripper_client = rospy.ServiceProxy('precision_gripper_command',PrecisionGripperCommand)
        rospy.sleep(.5)
        
        request = PrecisionGripperCommandRequest()
        request.close_outer_gripper_fully = True
        
        rospy.loginfo("Closing outer gripper")
        precision_gripper_client(request)
        rospy.loginfo("Waiting 5 seconds")
        rospy.sleep(1)

        request = PrecisionGripperCommandRequest()
        request.open_outer_gripper_fully = True
        request.close_outer_gripper_fully = False

        rospy.loginfo("Opening outer gripper")
        precision_gripper_client(request)
        rospy.loginfo("Waiting 5 seconds")
        rospy.sleep(1)

        request.stop = True
        request.open_outer_gripper_fully = False
        request.close_outer_gripper_fully = False

        rospy.loginfo("Disabling torque (stopping the gripper)")
        precision_gripper_client(request)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
