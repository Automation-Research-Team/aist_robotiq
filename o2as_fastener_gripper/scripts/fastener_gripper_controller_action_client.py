#! /usr/bin/env python

from __future__ import print_function
import sys
import rospy
from std_msgs.msg import String
from o2as_fastener_gripper.srv import *
from o2as_fastener_gripper.msg import *

import actionlib
import actionlib_tutorials.msg

class ControlClass:
    def contorol_client(self,name):
        client = actionlib.SimpleActionClient(name + '/Action', FastenerGripperControlAction)
    
        client.wait_for_server()
        goal = FastenerGripperControlGoal()
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('fastener_gripper_controller_test')

        parent = ControlClass()

        fastener_grippers = [
            "fg1",
            "fg2",
            "fg3",
        ]

        for c in fastener_grippers:
            parent.contorol_client(c)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)