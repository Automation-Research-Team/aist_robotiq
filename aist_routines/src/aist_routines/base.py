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
import copy

import rospy
import actionlib
import std_msgs.msg
import geometry_msgs.msg
import moveit_commander
from moveit_commander.conversions import pose_to_list
import robotiq_msgs.msg

import o2as_msgs.msg
import o2as_msgs.srv

def is_program_running(topic_namespace = ""):
    """Checks if a program is running on the UR"""
    msg = rospy.wait_for_message(topic_namespace + "/ur_driver/robot_mode_state", ur_modern_driver.msg.RobotModeDataMsg)
    if msg:
        return msg.is_program_running
    else:
        rospy.logerr("No message received from the robot. Is everything running? Is the namespace entered correctly with a leading slash?")
        return False

def wait_for_UR_program(topic_namespace = "", timeout_duration = rospy.Duration.from_sec(20.0)):
    rospy.logdebug("Waiting for UR program to finish.")
    # Only run this after sending custom URScripts and not the regular motion commands, or this call will not terminate before the timeout.
    rospy.sleep(1.0)
    t_start = rospy.Time.now()
    time_passed = rospy.Time.now() - t_start
    while is_program_running(topic_namespace):
        rospy.sleep(.05)
        time_passed = rospy.Time.now() - t_start
        if time_passed > timeout_duration:
            rospy.loginfo("Timeout reached.")
        return False
    rospy.logdebug("UR Program has terminated.")
    return True


class AISTBaseRoutines(object):
    def __init__(self):
        super(AISTBaseRoutines, self).__init__()
        rospy.init_node("aist_routines", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        self.goToNamedPose_client  = rospy.ServiceProxy(
                                        '/aist_skills/goToNamedPose',
                                        o2as_msgs.srv.goToNamedPose)
        self.goToPoseGoal_client   = rospy.ServiceProxy(
                                        '/aist_skills/goToPoseGoal',
                                        o2as_msgs.srv.goToPoseGoal)
        self.urscript_client       = rospy.ServiceProxy(
                                        '/o2as_skills/sendScriptToUR',
                                        o2as_msgs.srv.sendScriptToUR)
        self.gripperCommand_client = rospy.ServiceProxy(
                                        '/aist_skills/gripperCommand',
                                        o2as_msgs.srv.gripperCommand)

        self.pickOrPlace_client    = actionlib.SimpleActionClient(
                                        '/aist_skills/pick',
                                        o2as_msgs.msg.pickOrPlaceAction)

    def cycle_through_calibration_poses(self, poses, robot_name,
                                        speed=0.3, move_lin=False,
                                        go_home=True, end_effector_link=""):
        home_pose = "home"
        if "screw" in end_effector_link:
            home_pose = "screw_ready"

        for pose in poses:
            rospy.loginfo("============ Press `Enter` to move "
                          + robot_name + " to " + pose.header.frame_id)
            self.publish_marker(pose, "place_pose")
            raw_input()
            if go_home:
                self.go_to_named_pose(robot_name, home_pose)
            if rospy.is_shutdown():
                break
            else:
                self.go_to_pose_goal(robot_name, pose,speed=speed,
                                     end_effector_link=end_effector_link,
                                     move_lin=move_lin)

            rospy.loginfo("============ Press `Enter` to proceed ")
            raw_input()

        if go_home:
            rospy.loginfo("Moving all robots home again.")
            self.go_to_named_pose(robot_name, "home")
        return

    def go_to_named_pose(self, named_pose, group_name):
        req = o2as_msgs.srv.goToNamedPoseRequest()
        req.planning_group = group_name
        req.named_pose     = named_pose
        return self.goToNamedPose_client.call(req)

    def go_to_pose_goal(self, group_name, target_pose, speed=1.0,
                        high_precision=False, end_effector_link="",
                        move_lin=False):
        req = o2as_msgs.srv.goToPoseGoalRequest()
        req.planning_group    = group_name
        req.target_pose       = target_pose
        req.speed             = speed
        req.high_precision    = high_precision
        req.end_effector_link = end_effector_link
        req.move_lin          = move_lin
        return self.goToPoseGoal_client.call(req)

    def do_pick_action(self, group_name, pose_stamped):
        return self.pickOrPlace(group_name, pose_stamped, True)

    def do_place_action(self, group_name, pose_stamped):
        return self.pickOrPlace(group_name, pose_stamped, False)

    def pickOrPlace(self, group_name, pose_stamped, pick):
        goal = o2as_msgs.msg.pickOrPlaceGoal()
        goal.group_name      = group_name
        goal.pose            = pose_stamped
        goal.pick            = pick
        goal.approach_offset = 0.1
        goal.speed_fast      = 1.0
        goal.speed_slow      = 0.1
        self.pickOrPlace_client.send_goal(goal)
        self.pickOrPlace_client.wait_for_result()
        return self.pick_client.get_result()

    def send_gripper_command(self, group_name, command):
        req = o2as_msgs.srv.gripperCommandRequest()
        req.group_name = group_name
        req.command    = command
        return self.gripperCommand_client.call(req)

    # def do_linear_push(self, robot_name, force, wait=True, direction="Z+", max_approach_distance=0.1, forward_speed=0.0):
    #     if not self.use_real_robot:
    #         return True
    #     # Directly calls the UR service rather than the action of the skill_server
    #     req = o2as_msgs.srv.sendScriptToURRequest()
    #     req.robot_name = robot_name
    #     req.max_force = force
    #     req.force_direction = direction
    #     req.max_approach_distance = max_approach_distance
    #     req.forward_speed = forward_speed
    #     req.program_id = "linear_push"
    #     res = self.urscript_client.call(req)
    #     if wait:
    #         rospy.sleep(2.0)    # This program seems to take some time
    #         wait_for_UR_program("/" + robot_name +"_controller", rospy.Duration.from_sec(30.0))
    #     return res.success

    # FIXME: May move this function to aist_skills. Please check.
    def publish_marker(self, pose_stamped, marker_type):
        req = o2as_msgs.srv.publishMarkerRequest()
        req.marker_pose = pose_stamped
        req.marker_type = marker_type
        self.publishMarker_client.call(req)
        return True
