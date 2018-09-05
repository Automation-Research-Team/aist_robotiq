#! /usr/bin/env python

from __future__ import print_function
import sys
import rospy
from std_msgs.msg import String
from o2as_fastening_tools.srv import *
from o2as_fastening_tools.msg import *

import actionlib
import actionlib_tutorials.msg

class FasteningToolController:
    def fasten(self,name):
        client = actionlib.SimpleActionClient('FastenerGripperControlAction', FastenerGripperControlAction)
        client.wait_for_server()
        goal = FastenerGripperControlGoal()

        goal.fastening_tool_name = name
        goal.speed = 60

        client.send_goal_and_wait(goal,rospy.Duration(10),rospy.Duration(10))
        client.wait_for_result()
        
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('fastening_tool_controller_test')

        controller = FasteningToolController()

        name_list = [
            "m4_tool",
            "m3_tool",
        ]

        # rospy.sleep(3)

        for name in name_list:
            if not controller.fasten(name).control_result :
                break

        print('')

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
