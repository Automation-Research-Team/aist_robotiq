#!/usr/bin/env python
from __future__ import division, print_function
import sys
import copy

import rospy
import geometry_msgs.msg
import actionlib

import aist_skills.msg

if __name__ == "__main__":
    axis = sys.argv[1]
    distance = float(sys.argv[2])
    repeat_num = int(sys.argv[3])

    rospy.init_node('reciprocating_motion_test', anonymous=False)

    move_lin_action_client = actionlib.SimpleActionClient('aist_skills/move_lin', aist_skills.msg.MoveLinAction)

    group_name = 'a_bot'
    frame_id = 'o2as_ground'
    point = geometry_msgs.msg.Point(0, 0.3, 0.3)
    orientation = geometry_msgs.msg.Point(0, 90, 0)
    speed = 1.0

    goal = aist_skills.msg.MoveLinActionGoal()
    goal.group_name = group_name
    goal.frame_id = frame_id
    goal.orientation = orientation
    goal.point = point

    move_lin_action_client.send_goal_and_wait(goal)
    move_lin_action_client.send_goal_and_wait(goal)
    rospy.sleep(5)

    if axis == 'x':
        point.x += distance
        goal.point = point
        move_lin_action_client.send_goal_and_wait(goal)
        point.x -= distance
        goal.point = point
        move_lin_action_client.send_goal_and_wait(goal)
        point.x -= distance
        goal.point = point
        move_lin_action_client.send_goal_and_wait(goal)
        point.x += distance
        goal.point = point
        move_lin_action_client.send_goal_and_wait(goal)
