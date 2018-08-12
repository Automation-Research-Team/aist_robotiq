#!/usr/bin/env python

import precision_gripper_class as pgc
import time
import rospy
from o2as_precision_gripper.srv import *

if __name__ == "__main__":
    #initialise the class here
    gripper = pgc.PrecisionGripper()
    rospy.init_node("precision_gripper_server")
    my_service = rospy.Service('precision_gripper_command', PrecisionGripperCommand, gripper.my_callback)
    rospy.loginfo("Service precision_gripper is ready")
    rospy.spin()
