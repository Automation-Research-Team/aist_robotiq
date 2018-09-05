#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg

from o2as_routines.base import O2ASBaseRoutines

if __name__ == '__main__':

    baseRoutine = O2ASBaseRoutines()

    baseRoutine.go_to_named_pose("home", "c_bot")
    baseRoutine.go_to_named_pose("home", "b_bot")
    baseRoutine.go_to_named_pose("home", "a_bot")
    robot_name = "b_bot"
    baseRoutine.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    # position 1
    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.header.frame_id = "world"
    pose1.pose.position.z = .85
    pose1.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    baseRoutine.go_to_pose_goal(robot_name, pose1, speed=0.05)
