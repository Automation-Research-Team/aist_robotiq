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
    def screw(self, name, mode, state):
        client = actionlib.SimpleActionClient('o2as_fastening_tools/screw_control_action', ScrewControlAction)
        client.wait_for_server()
        goal = ScrewControlGoal()

        goal.fastening_tool_name = name
        goal.screw_control = mode
        goal.switch = state

        client.send_goal_and_wait(goal,rospy.Duration(30),rospy.Duration(10))
        client.wait_for_result()

        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('fastening_screw_test')

        controller = FasteningToolController()

        name_list = [
            "screw_tool_m5",
        ]

        for name in name_list:
            res = controller.screw(name, "Insert", False)

            if not res.control_result :
                rospy.logerr("Can not Insert screw")

            rospy.sleep(1)

    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)