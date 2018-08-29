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
# Author: Felix von Drigalski

# This file is based on the kinetic MoveIt tutorial for the Python movegroup interface.

import sys
import copy
import rospy
import tf_conversions
import tf 
import actionlib
from math import *

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import robotiq_msgs.msg
import std_srvs.srv

import o2as_msgs
import o2as_msgs.msg
import o2as_msgs.srv

from math import pi
from std_msgs.msg import String
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

class O2ASBaseRoutines(object):
  """
  This class contains the common helper and convenience functions used in the routines.
  The common functions include the initialization of the services and actions,
  and shorthand functions for the most common actions.
  """
  def __init__(self):
    # super(O2ASBaseRoutines, self).__init__()
    
    self.listener = tf.TransformListener()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('assembly_example', anonymous=False, log_level=rospy.DEBUG)

    self.robots = moveit_commander.RobotCommander()
    self.groups = {"a_bot":moveit_commander.MoveGroupCommander("a_bot"),
              "b_bot":moveit_commander.MoveGroupCommander("b_bot"),
              "c_bot":moveit_commander.MoveGroupCommander("c_bot"),
              "front_bots":moveit_commander.MoveGroupCommander("front_bots")}
              # "all_bots":moveit_commander.MoveGroupCommander("all_bots") }
    self.gripper_action_clients = { "a_bot":actionlib.SimpleActionClient('precision_gripper_action', o2as_msgs.msg.PrecisionGripperCommandAction), 
                               "b_bot":actionlib.SimpleActionClient('/b_bot_gripper/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction), 
                               "c_bot":actionlib.SimpleActionClient('/c_bot_gripper/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction) }

    self.pick_client = actionlib.SimpleActionClient('/o2as_skills/pick', o2as_msgs.msg.pickAction)
    self.place_client = actionlib.SimpleActionClient('/o2as_skills/place', o2as_msgs.msg.placeAction)
    self.regrasp_client = actionlib.SimpleActionClient('/o2as_skills/regrasp', o2as_msgs.msg.regraspAction)
    self.align_client = actionlib.SimpleActionClient('/o2as_skills/align', o2as_msgs.msg.alignAction)
    self.insert_client = actionlib.SimpleActionClient('/o2as_skills/insert', o2as_msgs.msg.insertAction)
    self.screw_client = actionlib.SimpleActionClient('/o2as_skills/screw', o2as_msgs.msg.screwAction)

    self.urscript_client = rospy.ServiceProxy('/o2as_skills/sendScriptToUR', o2as_msgs.srv.sendScriptToUR)
    self.goToNamedPose_client = rospy.ServiceProxy('/o2as_skills/goToNamedPose', o2as_msgs.srv.goToNamedPose)
    self.publishMarker_client = rospy.ServiceProxy('/o2as_skills/publishMarker', o2as_msgs.srv.publishMarker)
    self.toggleCollisions_client = rospy.ServiceProxy('/o2as_skills/toggleCollisions', std_srvs.srv.SetBool)

    # self.pick_client.wait_for_server() # wait for the clients to connect
    # self.place_client.wait_for_server() 
    # self.align_client.wait_for_server() 
    # self.insert_client.wait_for_server() 
    # self.screw_client.wait_for_server() 
    rospy.sleep(.5)   # 
    rospy.loginfo("Finished initializing class")
    
  ############## ------ Internal functions (and convenience functions)

  def publish_marker(self, pose_stamped, marker_type):
    req = o2as_msgs.srv.publishMarkerRequest()
    req.marker_pose = pose_stamped
    req.marker_type = marker_type
    self.publishMarker_client.call(req)
    return True

  def go_to_pose_goal(self, group_name, pose_goal_stamped, speed = 1.0, high_precision = False):
    group = self.groups[group_name]
    group.set_pose_target(pose_goal_stamped)
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)

    if high_precision:
      group.set_goal_tolerance(.000001)
      group.set_planning_time(10)

    plan = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    
    # Reset the precision
    if high_precision:
      group.set_goal_tolerance(.0001) 
      group.set_planning_time(3) 

    current_pose = group.get_current_pose().pose
    return all_close(pose_goal_stamped.pose, current_pose, 0.01)

  def move_front_bots(self, pose_goal_a_bot, pose_goal_b_bot, speed = 0.05):
    rospy.logwarn("CAUTION: Moving front bots together, but MoveIt does not do continuous collision checking.")
    group = self.groups["front_bots"]
    group.set_pose_target(pose_goal_a_bot, end_effector_link="a_bot_gripper_tip_link")
    group.set_pose_target(pose_goal_b_bot, end_effector_link="b_bot_robotiq_85_tip_link")
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)

    success = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    rospy.loginfo("Received:")
    rospy.loginfo(success)
    return success

  def belt_spiral_motion(self, robot_name, start_pose, speed = 0.02):
    group = self.groups[robot_name]
    rospy.loginfo("Performing belt spiral motion " + str(speed))
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)

    r_belt=0.0068
    theta_belt=0
    theta_increase=15
    
    # ==== MISBEHAVING VERSION (see https://answers.ros.org/question/300978/movegroupcommander-get_current_pose-returns-incorrect-result-when-using-real-robot/ )
    # start_pos_bugged = group.get_current_pose() 
    # ==== WORKING VERSION:
    gripper_pose = geometry_msgs.msg.PoseStamped()
    gripper_pose.header.frame_id = "a_bot_gripper_tip_link"
    gripper_pose.pose.orientation.w = 1.0
    # start_pose = self.listener.transformPose("world", gripper_pose)

    next_pose = start_pose
    while theta_belt <= 360 and not rospy.is_shutdown():
        #By default, the Spiral_Search function will maintain contact between both mating parts at all times
         theta_belt=theta_belt+theta_increase
         x=cos(radians(theta_belt))*r_belt
         y=sin(radians(theta_belt))*r_belt
         next_pose.pose.position.x = start_pose.pose.position.x + x
         next_pose.pose.position.y = start_pose.pose.position.y - y
         self.go_to_pose_goal(robot_name, next_pose)
         rospy.sleep(0.1)
      
    # -------------
    return True

  def horizontal_spiral_motion(self, robot_name, max_radius, start_pos, radius_increment = .001, speed = 0.02):
    group = self.groups[robot_name]
    rospy.loginfo("Performing horizontal spiral motion " + str(speed))
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)

    # Modified code from Robotiq spiral search
    theta_incr = 30
    radius_inc_set = radius_increment / (360 / theta_incr)
    r=0.0003  #Start radius
    theta=0
    RealRadius=0
    
    # ==== MISBEHAVING VERSION (see https://answers.ros.org/question/300978/movegroupcommander-get_current_pose-returns-incorrect-result-when-using-real-robot/ )
    # start_pos_bugged = group.get_current_pose() 
    # ==== WORKING VERSION:
    gripper_pos = geometry_msgs.msg.PoseStamped()
    gripper_pos.header.frame_id = "a_bot_gripper_tip_link"
    gripper_pos.pose.orientation.w = 1.0
    # start_pos = self.listener.transformPose("world", gripper_pos)

    next_pos = start_pos
    while RealRadius <= max_radius and not rospy.is_shutdown():
        #By default, the Spiral_Search function will maintain contact between both mating parts at all times
        theta=theta+theta_incr
        x=cos(radians(theta))*r
        y=sin(radians(theta))*r
        next_pos.pose.position.x = start_pos.pose.position.x + x
        next_pos.pose.position.y = start_pos.pose.position.y + y
        r=r + radius_inc_set
        RealRadius = sqrt(pow(x,2)+pow(y,2))
        self.go_to_pose_goal(robot_name, next_pos)
        rospy.sleep(0.1)
    # -------------
    return True

  def go_to_named_pose(self, pose_name, robot_name, speed = 1.0):
    # pose_name should be "home", "back" etc.
    self.groups[robot_name].set_named_target(pose_name)
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    self.groups[robot_name].set_max_velocity_scaling_factor(speed)
    self.groups[robot_name].go(wait=True)
    self.groups[robot_name].stop()
    self.groups[robot_name].clear_pose_targets()
    return True

  def do_pick_action(self, robot_name, pose_stamped, z_axis_rotation = 0.0, use_complex_planning = False, tool_name = ""):
    # Call the pick action
    goal = o2as_msgs.msg.pickGoal()
    goal.robot_name = robot_name
    goal.item_pose = pose_stamped
    goal.tool_name = tool_name
    goal.use_complex_planning = use_complex_planning
    goal.z_axis_rotation = z_axis_rotation
    rospy.loginfo("Sending pick action goal")
    rospy.loginfo(goal)

    self.pick_client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    self.pick_client.wait_for_result()
    rospy.loginfo("Getting result")
    return self.pick_client.get_result()

  def do_place_action(self, robot_name, pose_stamped, tool_name = ""):
    # Call the pick action
    goal = o2as_msgs.msg.placeGoal()
    goal.robot_name = robot_name
    goal.item_pose = pose_stamped
    goal.tool_name = tool_name
    rospy.loginfo("Sending place action goal")
    rospy.loginfo(goal)

    self.place_client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    self.place_client.wait_for_result()
    rospy.loginfo("Getting result")
    return self.place_client.get_result()

  def do_insert_action(self, active_robot_name, passive_robot_name = "", 
                        starting_offset = 0.05, max_insertion_distance=0.01, 
                        max_approach_distance = .1, max_force = 5,
                        max_radius = .001, radius_increment = .0001):
    goal = o2as_msgs.msg.insertGoal()
    goal.active_robot_name = active_robot_name
    goal.passive_robot_name = passive_robot_name
    goal.starting_offset = starting_offset
    goal.max_insertion_distance = max_insertion_distance
    goal.max_approach_distance = max_approach_distance
    goal.max_force = max_force
    goal.max_radius = max_radius
    goal.radius_increment = radius_increment
    rospy.loginfo("Sending insert action goal.")    
    self.insert_client.send_goal(goal)
    self.insert_client.wait_for_result()
    return self.insert_client.get_result()

  def do_insertion(self, robot_name):
    # Directly calls the UR service rather than the action of the skill_server
    req = o2as_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.program_id = "insertion"
    res = self.urscript_client.call(req)
    return res.success
  
  def do_linear_push(self, robot_name):
    # Directly calls the UR service rather than the action of the skill_server
    req = o2as_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.program_id = "linear_push"
    res = self.urscript_client.call(req)
    return res.success

  def do_regrasp(self, giver_robot_name, receiver_robot_name, grasp_distance = .02):
    """The item goes from giver to receiver."""
    goal = o2as_msgs.msg.regraspGoal()
    goal.giver_robot_name = giver_robot_name
    goal.receiver_robot_name = receiver_robot_name
    goal.grasp_distance = grasp_distance

    self.regrasp_client.send_goal(goal)
    rospy.loginfo("Performing regrasp with grippers " + giver_robot_name + " and " + receiver_robot_name)
    self.regrasp_client.wait_for_result(rospy.Duration(90.0))
    result = self.regrasp_client.get_result()
    return result

  def toggle_collisions(self, collisions_on):
    req = std_srvs.srv.SetBoolRequest()
    req.data = collisions_on
    res = self.toggleCollisions_client.call(req)
    return res.success


  ################ ----- Gripper interfaces
  
  def send_gripper_command(self, gripper, command, this_action_grasps_an_object = False):
    if gripper == "precision_gripper_outer" or gripper == "precision_gripper_inner":
      goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
      if command == "stop":
        goal.stop = True
      elif command == "close":
        if gripper == "precision_gripper_inner":
          goal.close_inner_gripper_fully = True
        else:
          goal.close_outer_gripper_fully = True
      elif command == "open":
        if gripper == "precision_gripper_inner":
          goal.open_inner_gripper_fully = True
        else:
          goal.open_outer_gripper_fully = True
      goal.this_action_grasps_an_object = this_action_grasps_an_object
      action_client = self.gripper_action_clients["a_bot"]
    elif gripper == "b_bot" or gripper == "c_bot":
      goal = robotiq_msgs.msg.CModelCommandGoal()
      action_client = self.gripper_action_clients[gripper]
      goal.velocity = 0.1   # from 0.013 to 0.1
      if command == "close":
        goal.position = 0.0
      elif command == "open":
        goal.position = 0.085
      else:
        goal.position = command     # This sets the opening width directly
    else:
      rospy.logerr("Could not parse gripper command")

    action_client.send_goal(goal)
    rospy.loginfo("Sending command " + command + " to gripper: " + gripper)
    action_client.wait_for_result(rospy.Duration(3.0))  # Default wait time: 3 s
    result = action_client.get_result()
    rospy.loginfo(result)
    return 

  def precision_gripper_outer_close(self):
    try:
        goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        goal.close_outer_gripper_fully = True
        self.gripper_action_clients["c_bot"].send_goal(goal)
        rospy.loginfo("close outer gripper")
        self.gripper_action_clients["c_bot"].wait_for_result()
        result = self.gripper_action_clients["c_bot"].get_result()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)


  def precision_gripper_outer_open(self):
    try:
        goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        goal.open_outer_gripper_fully = True
        goal.close_outer_gripper_fully = False
        self.gripper_action_clients["c_bot"].send_goal(goal)
        rospy.loginfo("open outer gripper")
        self.gripper_action_clients["c_bot"].wait_for_result()
        result = self.gripper_action_clients["c_bot"].get_result()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)

  def precision_gripper_inner_close(self, this_action_grasps_an_object = False):
    try:
        goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        goal.close_inner_gripper_fully = True
        goal.this_action_grasps_an_object = this_action_grasps_an_object
        self.gripper_action_clients["c_bot"].send_goal(goal)
        rospy.loginfo("Closing inner gripper")
        self.gripper_action_clients["c_bot"].wait_for_result()
        result = self.gripper_action_clients["c_bot"].get_result()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)


  def precision_gripper_inner_open(self, this_action_grasps_an_object = False):
    try:
        goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        goal.open_inner_gripper_fully = True
        goal.close_inner_gripper_fully = False
        goal.this_action_grasps_an_object = this_action_grasps_an_object
        self.gripper_action_clients["c_bot"].send_goal(goal)
        rospy.loginfo("Opening inner gripper")
        self.gripper_action_clients["c_bot"].wait_for_result()
        result = self.gripper_action_clients["c_bot"].get_result()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

