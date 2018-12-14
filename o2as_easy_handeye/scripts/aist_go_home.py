#!/usr/bin/env python

import sys
import rospy
from aist_routines.base import AISTBaseRoutines


if __name__ == '__main__':

    robot_name   = sys.argv[1]
    assert(robot_name in {"a_bot", "b_bot"})

    f = AISTBaseRoutines
    rospy.init_node("aist_go_home")
    baseRoutines = f()
    baseRoutines.go_to_named_pose("home", robot_name)
