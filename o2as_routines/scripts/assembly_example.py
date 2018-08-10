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
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import o2as_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL

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

class ExampleClass(object):
  """ExampleClass"""
  def __init__(self):
    super(ExampleClass, self).__init__()
    
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    self.robots = moveit_commander.RobotCommander()
    # group_name = rospy.get_param("move_group_name", "a_bot")
    # rospy.loginfo(group_name)
    # ee_link = rospy.get_param("ee_link", "a_bot_robotiq_85_tip_link")
    # rospy.loginfo(ee_link)
    # group = moveit_commander.MoveGroupCommander(group_name)
    self.groups = {"a_bot":moveit_commander.MoveGroupCommander("a_bot"),
              "b_bot":moveit_commander.MoveGroupCommander("b_bot"),
              "c_bot":moveit_commander.MoveGroupCommander("c_bot"),
              "front_bots":moveit_commander.MoveGroupCommander("front_bots"),
              "all_bots":moveit_commander.MoveGroupCommander("all_bots") }
    self.gripper_action_clients = { "a_bot":actionlib.SimpleActionClient('/a_bot_gripper/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction), 
                               "b_bot":actionlib.SimpleActionClient('/b_bot_gripper/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction), 
                               "c_bot":actionlib.SimpleActionClient('/c_bot_gripper/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction) }

    self.pick_client = actionlib.SimpleActionClient('/o2as_skills/pick', o2as_skills.msg.PickAction)
    self.place_client = actionlib.SimpleActionClient('/o2as_skills/place', o2as_skills.msg.PlaceAction)
    self.align_client = actionlib.SimpleActionClient('/o2as_skills/align', o2as_skills.msg.AlignAction)
    self.insert_client = actionlib.SimpleActionClient('/o2as_skills/insert', o2as_skills.msg.InsertAction)
    self.screw_client = actionlib.SimpleActionClient('/o2as_skills/screw', o2as_skills.msg.ScrewAction)

    self.urscript_client = rospy.ServiceProxy('/o2as_skills/ur_program_relay', o2as_skills.msg.sendScriptToUR)
    self.goToNamedPose_client = rospy.ServiceProxy('/o2as_skills/goToNamedPose', o2as_skills.msg.goToNamedPose)

  def go_to_pose_goal(self, group_name, pose_goal_stamped):
    group = self.groups[group_name]
    group.set_pose_target(pose_goal_stamped)  # How to set multiple goals? Does not seem to work in Python.

    ## Now, we call the planner to compute the plan and execute it.
    plan = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal_stamped.pose, current_pose, 0.01)

  def do_pick_action(self, robot_name, pose_stamped, link_name = "ee_link"):
    # Call the pick action
    return True

  def do_insertion(self, robot_name):
    srv = o2as_skills.msg.sendScriptToUR
    srv.request.robot_name = robot_name
    srv.request.program_id = "insertion"
    self.urscript_client.call(srv)
    return srv.response.success

  def assembly_demo(self):
    # Pick a thing with b_bot, then c_bot, then send the insertion command to c_bot
    
    # Define the pose of each item (ee_link)
    peg_pose = geometry_msgs.msg.PoseStamped()
    peg_pose.header.frame_id = "c_bot_base"
    peg_pose.pose.orientation.x = -0.5
    peg_pose.pose.orientation.y = 0.5
    peg_pose.pose.orientation.z = 0.5
    peg_pose.pose.orientation.w = 0.5
    peg_pose.pose.position.x = 0.045
    peg_pose.pose.position.y = -0.300
    peg_pose.pose.position.z = 0.254
    
    bearing_pose = geometry_msgs.msg.PoseStamped()
    bearing_pose.header.frame_id = "b_bot_base"
    bearing_pose.pose.orientation.x = -0.5
    bearing_pose.pose.orientation.y = 0.5
    bearing_pose.pose.orientation.z = 0.5
    bearing_pose.pose.orientation.w = 0.5
    bearing_pose.pose.position.x = -0.025
    bearing_pose.pose.position.y = -0.658
    bearing_pose.pose.position.z = 0.233
    
    # First use a_bot

    do_pick_action("b_bot", bearing_pose, link_name = "ee_link")
    
    if not success:
      # Add a line to the log here
      rospy.loginfo("Something went wrong")
    
    # Move the robot back (there will be a more convenient function for this later)
    home_pose = geometry_msgs.msg.PoseStamped()
    home_pose.pose = pose_goal.pose.orientation
    home_pose.pose.position.x = -0.4
    home_pose.pose.position.y = 0.4
    home_pose.pose.position.z = 0.4
    home_pose.header.frame_id = "a_bot_base_link"
    self.go_to_pose_goal(home_pose)

    # Now check with b_bot
    self.group = moveit_commander.MoveGroupCommander("b_bot")
    success = True

    bin_header_ids = ['/set2_bin1', '/set2_bin2', '/set2_bin3']
    for bin_id in bin_header_ids:
      pose_goal.header.frame_id = bin_id
      rospy.loginfo("Trying to move b_bot to bin:" + bin_id)
      if self.go_to_pose_goal(pose_goal):
        rospy.sleep(2)
      else:
        success = False
  
    if not success:
      # Add a line to the log here
      rospy.loginfo("Something went wrong")


def main():
  try:
    tutorial = ExampleClass()

    print "============ Press `Enter` to go to different bin positions ..."
    raw_input()
    # tutorial.cycle_through_bins()
    tutorial.check_all_bins()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
