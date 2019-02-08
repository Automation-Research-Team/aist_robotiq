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
import tf
import tf_conversions
import moveit_commander
from moveit_commander.conversions import pose_to_list
import ur_modern_driver.msg

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
        rospy.init_node("kitting_task", anonymous=True)

        self.robots = moveit_commander.RobotCommander()
        self.planning_scene = moveit_commander.PlanningSceneInterface()
        self.groups = {
            "a_bot": moveit_commander.MoveGroupCommander("a_bot"),
            "b_bot": moveit_commander.MoveGroupCommander("b_bot")
        }
        self.listener = tf.TransformListener()
        self.publishMarker_client = rospy.ServiceProxy('/aist_skills/publishMarker', o2as_msgs.srv.publishMarker)
        self.urscript_client = rospy.ServiceProxy('/o2as_skills/sendScriptToUR', o2as_msgs.srv.sendScriptToUR)
        self.setup_suction_tool()

        self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
        self.downward_orientation_a_bot = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2,pi/2,0))

        # For debugging of move_lin
        self.actual_pose_pub = rospy.Publisher('actual_pose', geometry_msgs.msg.Pose, queue_size=10)


    def setup_suction_tool(self):
        """Enable to use suction tool."""
        self._suction = actionlib.SimpleActionClient('o2as_fastening_tools/suction_control', o2as_msgs.msg.SuctionControlAction)
        self._suctioned = False
        self._suction_state = rospy.Subscriber("suction_tool/screw_suctioned", std_msgs.msg.Bool, self._suction_state_callback)


    def cycle_through_calibration_poses(self, poses, robot_name, speed=0.3, move_lin=False, go_home=True, end_effector_link=""):
        home_pose = "home"
        if "screw" in end_effector_link:
            home_pose = "screw_ready"

        for pose in poses:
            rospy.loginfo("============ Press `Enter` to move " + robot_name + " to " + pose.header.frame_id)
            self.publish_marker(pose, "place_pose")
            raw_input()
            if go_home:
                self.go_to_named_pose(home_pose, robot_name)
            if rospy.is_shutdown():
                break
            else:
                self.go_to_pose_goal(robot_name, pose,speed=speed, end_effector_link=end_effector_link, move_lin = move_lin)

            rospy.loginfo("============ Press `Enter` to proceed ")
            raw_input()

            # if go_home:
            #     self.go_to_named_pose(home_pose, robot_name, force_ur_script=move_lin)

        if go_home:
            rospy.loginfo("Moving all robots home again.")
            self.go_to_named_pose("home", "b_bot")
        return


    def pick(self, robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, special_pick = False, lift_up_after_pick=True, timeout=3.0):
        self.publish_marker(object_pose, "pick_pose")
        if speed_fast > 1.0:
            acceleration=speed_fast
        else:
            acceleration=1.0

        approach_pose = copy.deepcopy(object_pose)
        rospy.logdebug("Approach height 0: " + str(approach_height))
        approach_pose.pose.position.z = approach_height
        rospy.logdebug("Going to height " + str(approach_pose.pose.position.z))
        self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast, move_lin=True)

        if gripper_command=="complex_pick_from_inside":
            self.precision_gripper_inner_close()
        elif gripper_command=="complex_pick_from_outside":
            self.precision_gripper_inner_open()
        elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
            self.precision_gripper_inner_close()
        elif gripper_command=="easy_pick_outside_only_inner" or gripper_command=="inner_gripper_from_outside":
            self.precision_gripper_inner_open()
        elif gripper_command=="suction":
            suck_res = self.suck(turn_suction_on=True, timeout=timeout)
        elif gripper_command=="none":
            pass
        else:
            self.send_gripper_command(gripper=robot_name, command="open")

        rospy.loginfo("Moving down to object")
        rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
        # if gripper_command=="suction":
        #     self.do_linear_push(robot_name, force=5.0, wait=True, direction='Z+', max_approach_distance=0.92, forward_speed=0.1)
        # else:
        #     self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, high_precision=True, move_lin=True)
        self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, high_precision=True, move_lin=True)


        #gripper close
        if gripper_command=="complex_pick_from_inside":
            self.precision_gripper_inner_open(this_action_grasps_an_object = True)
            self.precision_gripper_outer_close()
        elif gripper_command=="complex_pick_from_outside":
            self.precision_gripper_inner_close(this_action_grasps_an_object = True)
            self.precision_gripper_outer_close()
        elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
            self.precision_gripper_inner_open(this_action_grasps_an_object = True)
        elif gripper_command=="easy_pick_outside_only_inner" or gripper_command=="inner_gripper_from_outside":
            self.precision_gripper_inner_close(this_action_grasps_an_object = True)
        elif gripper_command=="suction":
            pass
        elif gripper_command=="none":
            pass
        else:
            self.send_gripper_command(gripper=robot_name, command="close")

        if lift_up_after_pick:
            rospy.sleep(1.0)
            rospy.loginfo("Going back up")
            rospy.loginfo("Going to height " + str(approach_pose.pose.position.z))
            self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast, move_lin=True)
        return True

    def place(self,robot_name, object_pose, place_height=None, speed_fast=1.0, speed_slow=0.05, gripper_command="", approach_height=0.05, lift_up_after_place=True):
        if speed_fast > 1.0:
            acceleration=speed_fast
        else:
            acceleration=1.0

        approach_pose = copy.deepcopy(object_pose)
        approach_pose.pose.position.z = approach_height
        self.publish_marker(object_pose, "place_pose")
        rospy.loginfo("Going above place target")
        self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast, acceleration=acceleration, move_lin=True)

        if place_height is not None:
            object_pose.pose.position.z = place_height
        rospy.loginfo("Moving to place target")
        self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, acceleration=acceleration, move_lin=True)

        #gripper open
        if gripper_command=="complex_pick_from_inside":
            self.precision_gripper_outer_open()
            self.precision_gripper_inner_close()
        elif gripper_command=="complex_pick_from_outside":
            self.precision_gripper_outer_open()
            self.precision_gripper_inner_open()
        elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
            self.precision_gripper_inner_close()
        elif gripper_command=="easy_pick_outside_only_inner" or gripper_command=="inner_gripper_from_outside":
            self.precision_gripper_inner_open()
        elif gripper_command=="suction":
            self.suck(turn_suction_on=False)
        elif gripper_command=="none":
            pass
        else:
            self.send_gripper_command(gripper=robot_name, command="open")


        if lift_up_after_place:
            rospy.loginfo("Moving back up")
            self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast, acceleration=acceleration, move_lin=True)
        return True

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

    def go_to_pose_goal(self, group_name, pose_goal_stamped, speed=1.0, acceleration=0.0, high_precision=False,
                        end_effector_link="", move_lin=True):
        if move_lin:
            return self.move_lin(group_name, pose_goal_stamped, speed, acceleration, end_effector_link)
        self.publish_marker(pose_goal_stamped, "pose")
        group = self.groups[group_name]

        if end_effector_link == "":
            if group_name == 'a_bot':
                end_effector_link = 'a_bot_robotiq_85_tip_link'
            elif group_name == "b_bot":
                end_effector_link = "b_bot_single_suction_gripper_pad_link"
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
        # self.publish_marker(pose_goal_stamped, "pose")

        if end_effector_link == "":
            if group_name == 'a_bot':
                end_effector_link = 'a_bot_robotiq_85_tip_link'
            elif group_name == "b_bot":
                end_effector_link = "b_bot_single_suction_gripper_pad_link"

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

        # current_pose = group.get_current_pose().pose

        current_pose_in_world = group.get_current_pose()
        current_pose_in_workspace = self.listener.transformPose('workspace_center', current_pose_in_world)
        self.actual_pose_pub.publish(current_pose_in_workspace.pose)
        rospy.loginfo("Target position: ")
        rospy.loginfo(pose_goal_stamped)
        rospy.loginfo("Current position: ")
        rospy.loginfo(current_pose_in_workspace)
        return plan_success

    def suck(self, turn_suction_on=False, eject=False, timeout=2.0):
        """Judge success or fail using pressure status."""
        if not self.use_real_robot:
            return True

        if turn_suction_on and eject:
            rospy.logwarn("Warning: Unexpected action might occur because suction and blow is both on.")
            return False

        goal = o2as_msgs.msg.SuctionControlGoal()
        goal.fastening_tool_name = "suction_tool"
        goal.turn_suction_on = turn_suction_on
        # goal.eject_screw = eject
        goal.eject_screw = False
        self._suction.send_goal(goal)
        self._suction.wait_for_result(rospy.Duration(timeout))
        return self._suction.get_result()

    def _suction_state_callback(self, msg):
        self._suctioned = msg.data

    def do_linear_push(self, robot_name, force, wait=True, direction="Z+", max_approach_distance=0.1, forward_speed=0.0):
        if not self.use_real_robot:
            return True
        # Directly calls the UR service rather than the action of the skill_server
        req = o2as_msgs.srv.sendScriptToURRequest()
        req.robot_name = robot_name
        req.max_force = force
        req.force_direction = direction
        req.max_approach_distance = max_approach_distance
        req.forward_speed = forward_speed
        req.program_id = "linear_push"
        res = self.urscript_client.call(req)
        if wait:
            rospy.sleep(2.0)    # This program seems to take some time
            wait_for_UR_program("/" + robot_name +"_controller", rospy.Duration.from_sec(30.0))
        return res.success

    # FIXME: May move this function to aist_skills. Please check.
    def publish_marker(self, pose_stamped, marker_type):
        req = o2as_msgs.srv.publishMarkerRequest()
        req.marker_pose = pose_stamped
        req.marker_type = marker_type
        self.publishMarker_client.call(req)
        return True
