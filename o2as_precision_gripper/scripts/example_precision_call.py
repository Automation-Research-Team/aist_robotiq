#!/usr/bin/env python

import sys
import rospy
from o2as_precision_gripper.srv import *

if __name__ == "__main__":
    
    rospy.wait_for_service('precision_gripper_command')
    print "kaidi"
    try:
        precision_gripper_client = rospy.ServiceProxy('precision_gripper_command',PrecisionGripperCommand)
        
        request = PrecisionGripperCommand()
        request.open_outer_gripper_fully = True
        # request.close_outer_gripper_fully = False
        # request.open_inner_gripper_fully = False
        # request.close_inner_gripper_fully = False
        
        response1 = precision_gripper_client(request)
        rospy.loginfo("response1.success = " + str(response1.success))
        print response1.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
