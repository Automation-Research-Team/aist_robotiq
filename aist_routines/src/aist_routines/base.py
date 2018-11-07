#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Yuma Hijioka

# This file is based on moveit tutorial for the kinetic MoveIt tutorial for the Python movegroup interface.or python.

from math import pi
import sys

import rospy
import tf
import tf_conversions
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class AISTBaseRoutines(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robots = moveit_commander.RobotCommander()
        self.planning_scene = moveit_commander.PlanningSceneInterface()
        self.groups = {
            "b_bot": moveit_commander.MoveGroupCommander("b_bot")
        }
        self.listener = tf.TransformListener()

    def go_to_named_pose(self, pose_name, robot_name, speed = 1.0, acceleration = 0.0):
        # pose_name should be "home", "back" etc.
        if speed > 1.0:
            speed = 1.0
        self.groups[robot_name].set_named_target(pose_name)
        rospy.logdebug("Setting velocity scaling to " + str(speed))
        self.groups[robot_name].set_max_velocity_scaling_factor(speed)
        self.groups[robot_name].go(wait=True)
        self.groups[robot_name].clear_pose_targets()
        return True

    def go_to_pose_goal(self, group_name, pose_goal_stamped, speed = 1.0, acceleration = 0.0, high_precision = False,
                        end_effector_link = "", move_lin = True):
        if move_lin:
            return self.move_lin(group_name, pose_goal_stamped, speed, acceleration, end_effector_link)
        self.publish_marker(pose_goal_stamped, "pose")
        group = self.groups[group_name]

        if not end_effector_link:
            if group_name == "b_bot":[]
                end_effector_link = "b_bot_suction_tool_tip_link"
        group.set_end_effector_link(end_effector_link)

        group.set_pose_target(pose_goal_stamped)
        rospy.logdebug("Setting velocity scaling to " + str(speed))
        group.set_max_velocity_scaling_factor(speed)

        if high_precision:
            group.set_goal_tolerance(.000001)
            group.set_planning_time(10)

        move_success = group.go(wait=True)
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        # Reset the precision
        if high_precision:
            group.set_goal_tolerance(.0001)
            group.set_planning_time(3)

        current_pose = group.get_current_pose().pose
        return all_close(pose_goal_stamped.pose, current_pose, 0.01), move_success

    def move_lin(self, group_name, pose_goal_stamped, speed = 1.0, acceleration = 0.0, end_effector_link = ""):
        self.publish_marker(pose_goal_stamped, "pose")

        if not end_effector_link:
            if group_name == "b_bot":
                end_effector_link = "b_bot_suction_tool_tip_link"

        group = self.groups[group_name]

        group.set_end_effector_link(end_effector_link)
        group.set_pose_target(pose_goal_stamped)
        rospy.logdebug("Setting velocity scaling to " + str(speed))
        group.set_max_velocity_scaling_factor(speed)


        # FIXME: At the start of the program, get_current_pose() did not return the correct value. Should be a bug report.
        waypoints = []
        pose_goal_world = self.listener.transformPose("world", pose_goal_stamped).pose
        waypoints.append(pose_goal_world)
        (plan, fraction) = group.compute_cartesian_path(waypoints,  # waypoints to follow
                                                        0.01,       # eef_step
                                                        0.0)        # jump_threshold
        rospy.loginfo("Compute cartesian path succeeded with " + str(fraction*100) + "%")
        plan = group.retime_trajectory(self.robots.get_current_state(), plan, speed)

        plan_success = group.execute(plan, wait=True)
        group.stop()
        group.clear_pose_targets()

        current_pose = group.get_current_pose().pose
        return plan_success
