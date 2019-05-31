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
import aist_msgs.msg
import aist_msgs.srv

def is_program_running(topic_namespace = ""):
    """Checks if a program is running on the UR"""
    msg = rospy.wait_for_message(topic_namespace + "/ur_driver/robot_mode_state", ur_modern_driver.msg.RobotModeDataMsg)
    if msg:
        return msg.is_program_running
    else:
        rospy.logerr("No message received from the robot. Is everything running? Is the namespace entered correctly with a leading slash?")
        return False

def wait_for_UR_program(topic_namespace="",
                        timeout_duration=rospy.Duration.from_sec(20.0)):
    rospy.logdebug("Waiting for UR program to finish.")
    # Only run this after sending custom URScripts and not the regular
    # motion commands, or this call will not terminate before the timeout.
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

        # Service clients
        self.goToNamedPose  = rospy.ServiceProxy('/aist_skills/goToNamedPose',
                                                 o2as_msgs.srv.goToNamedPose)
        self.goToPoseGoal   = rospy.ServiceProxy('/aist_skills/goToPoseGoal',
                                                 aist_msgs.srv.goToPoseGoal)
        self.sendScriptToUR = rospy.ServiceProxy('/o2as_skills/sendScriptToUR',
                                                 o2as_msgs.srv.sendScriptToUR)
        self.getGripperInfo = rospy.ServiceProxy('/aist_skills/getGripperInfo',
                                                 aist_msgs.srv.getGripperInfo)
        self.commandGripper = rospy.ServiceProxy('/aist_skills/commandGripper',
                                                 aist_msgs.srv.commandGripper)
        self.getCameraInfo  = rospy.ServiceProxy('/aist_skills/getCameraInfo',
                                                 aist_msgs.srv.getCameraInfo)
        self.commandCamera  = rospy.ServiceProxy('/aist_skills/commandCamera',
                                                 aist_msgs.srv.commandCamera)

        # Action clients
        self.pickOrPlace    = actionlib.SimpleActionClient(
                                        '/aist_skills/pickOrPlace',
                                        aist_msgs.msg.pickOrPlaceAction)
        self.pickOrPlace.wait_for_server()

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
        return self.goToNamedPose(group_name, named_pose)

    def go_to_pose_goal(self, robot_name, target_pose, speed=1.0,
                        high_precision=False, end_effector_link="",
                        move_lin=False):
        return self.goToPoseGoal(robot_name, target_pose, speed,
                                 high_precision, end_effector_link, move_lin)

    # Gripper stuffs
    def get_gripper_info(self, robot_name):
        res = self.getGripperInfo(robot_name)
        return (res.gripper_type, res.base_link, res.tip_link, res.success)

    def pregrasp(self, robot_name, command=""):
        return self.commandGripper(robot_name, 0, command).success

    def grasp(self, robot_name, command=""):
        return self.commandGripper(robot_name, 1, command).success

    def release(self, robot_name, command=""):
        return self.commandGripper(robot_name, 2, command).success

    # Camera stuffs
    def get_camera_info(self, camera_name):
        res = self.getCameraInfo(camera_name)
        return (res.camera_type, res.camera_info_topic,
                res.image_topic, res.success)

    def start_acquisition(self, camera_name):
        return self.commandCamera(camera_name, True).success

    def stop_acquisition(self, camera_name):
        return self.commandCamera(camera_name, False).success

    # Various actions
    def pick(self, robot_name, pose_stamped, grasp_offset=0.0,
             gripper_command="close",
             speed_fast=1.0, speed_slow=0.1, approach_offset=0.05,
             liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        return self._pick_or_place(robot_name, pose_stamped, True,
                                   gripper_command,
                                   grasp_offset, approach_offset, liftup_after,
                                   speed_fast, speed_slow, acc_fast, acc_slow)

    def place(self, robot_name, pose_stamped, grasp_offset=0.0,
              gripper_command="open",
              speed_fast=1.0, speed_slow=0.1, approach_offset=0.05,
              liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        return self._pick_or_place(robot_name, pose_stamped, False,
                                   gripper_command,
                                   grasp_offset, approach_offset, liftup_after,
                                   speed_fast, speed_slow, acc_fast, acc_slow)

    def _pick_or_place(self, robot_name, pose_stamped, pick, gripper_command,
                       grasp_offset, approach_offset, liftup_after,
                       speed_fast, speed_slow, acc_fast, acc_slow):
        goal = aist_msgs.msg.pickOrPlaceGoal()
        goal.robot_name      = robot_name
        goal.pose            = pose_stamped
        goal.pick            = pick
        goal.gripper_command = gripper_command
        goal.grasp_offset    = grasp_offset
        goal.approach_offset = approach_offset
        goal.liftup_after    = liftup_after
        goal.speed_fast      = speed_fast
        goal.speed_slow      = speed_slow
        goal.acc_fast        = acc_fast
        goal.acc_slow        = acc_slow
        self.pickOrPlace.send_goal(goal)
        self.pickOrPlace.wait_for_result()
        return self.pickOrPlace.get_result()

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
    #     res = self.urscript.call(req)
    #     if wait:
    #         rospy.sleep(2.0)    # This program seems to take some time
    #         wait_for_UR_program("/" + robot_name +"_controller", rospy.Duration.from_sec(30.0))
    #     return res.success

    # FIXME: May move this function to aist_skills. Please check.
    def publish_marker(self, pose_stamped, marker_type):
        req = o2as_msgs.srv.publishMarkerRequest()
        req.marker_pose = pose_stamped
        req.marker_type = marker_type
        self.publishMarker.call(req)
        return True
