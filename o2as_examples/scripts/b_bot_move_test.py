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

    baseRoutines = O2ASBaseRoutines()
    robot_name   = "b_bot"
    baseRoutines.go_to_named_pose("home", robot_name)
    baseRoutines.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    # position 1
    poseStamped = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id  = "world"
    poseStamped.pose.position.z  = .85
    poseStamped.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    baseRoutines.go_to_pose_goal(robot_name, poseStamped, speed=0.05)

    baseRoutines.go_to_named_pose("home", robot_name)
