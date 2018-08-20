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

import sys
import copy
import rospy

import geometry_msgs.msg
import tf_conversions
from math import pi

from o2as_routines.base import O2ASBaseRoutines

class CalibrationClass(O2ASBaseRoutines):
  """
  This routine checks the robot calibration by moving them to
  objects defined in the scene.
  """
  
  def check_robot_calibration(self):
    rospy.loginfo("============ Testing robot calibration. ============")
    rospy.loginfo("Each robot will move to a position in front of c_bot.")
    calib_pose = geometry_msgs.msg.PoseStamped()
    calib_pose.header.frame_id = "workspace_center"
    calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    calib_pose.pose.position.x = -0.15
    calib_pose.pose.position.z = 0.07

    self.go_to_named_pose("home_a", "a_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_c", "c_bot")

    rospy.loginfo("============ Press `Enter` to move a_bot to calibration position, enter 0 to skip.")
    if raw_input() != "0":
      self.send_gripper_command("precision_gripper_outer", "close")
      self.go_to_pose_goal("a_bot", calib_pose)

    rospy.loginfo("============ Press `Enter` to move b_bot to calibration position, enter 0 to skip.")
    if raw_input() != "0":
      self.go_to_named_pose("home_a", "a_bot")
      self.send_gripper_command("b_bot", "close")
      self.go_to_pose_goal("b_bot", calib_pose)

    rospy.loginfo("============ Press `Enter` to move c_bot to calibration position, enter 0 to skip.")
    if raw_input() != "0":
      self.go_to_named_pose("home_b", "b_bot")
      self.send_gripper_command("c_bot", "close")
      self.go_to_pose_goal("c_bot", calib_pose)

    rospy.loginfo("============ Press `Enter` to move robots back home.")
    raw_input()
    self.go_to_named_pose("home_a", "a_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_c", "c_bot")
    return
  
  def taskboard_calibration(self):
    rospy.loginfo("============ Calibrating taskboard. ============")
    poses = []

    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.header.frame_id = "taskboard_corner2"
    pose1.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose1.pose.position.z = .00

    pose2 = copy.deepcopy(pose1)
    pose2.header.frame_id = "taskboard_corner3"
    pose3 = copy.deepcopy(pose1)
    pose3.header.frame_id = "taskboard_part10"
    
    poses = [pose1, pose2, pose3]
    
    self.cycle_through_calibration_poses(poses, "a_bot", speed=0.3)
    return

  def taskboard_calibration_extended(self):
    rospy.loginfo("============ Demonstrating the calibration of the taskboard. ============")
    rospy.loginfo("This moves to the top of the parts pre-mounted in the taskboard.")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.z = 0.0

    # On top of the metal sheet
    pose1 = copy.deepcopy(pose0)
    pose1.header.frame_id = "taskboard_part7b"
    pose1.pose.position.y = .0015
    pose1.pose.position.z = .0253 + 0.0

    pose2 = copy.deepcopy(pose0)
    pose2.header.frame_id = "taskboard_part8"
    pose3 = copy.deepcopy(pose0)
    pose3.header.frame_id = "taskboard_part9"
    pose4 = copy.deepcopy(pose0)
    pose4.header.frame_id = "taskboard_part14"
    
    poses = [pose1, pose2, pose3, pose4]
    
    self.cycle_through_calibration_poses(poses, "a_bot", speed=0.3)
    return

  def taskboard_calibration_mat(self):
    rospy.loginfo("============ Calibrating placement mat for the taskboard task. ============")
    rospy.loginfo("Robot gripper tip should be 3 mm above the surface.")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.z = .003

    pose1 = copy.deepcopy(pose0)
    pose1.header.frame_id = "mat_part15"
    pose2 = copy.deepcopy(pose0)
    pose2.header.frame_id = "mat_part7b"
    pose3 = copy.deepcopy(pose0)
    pose3.header.frame_id = "mat_part3"
    
    poses = [pose1, pose2, pose3]

    self.cycle_through_calibration_poses(poses, "a_bot", speed=0.3)
    return 

  def gripper_frame_calibration_mat(self):
    rospy.loginfo("============ Calibrating the gripper tip frame for a_bot. ============")
    rospy.loginfo("Each approach of the target position has its orientation turned by 90 degrees.")
    poses = []

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "taskboard_part14"  # Good for demonstration, but not for calculation
    # pose0.header.frame_id = "mat_part10"      # Good for touching down and noting the position
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = -.002
    pose0.pose.position.y = -.002
    pose0.pose.position.z = .001

    q0 = tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0)
    q_turn_90 = tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0)
    q1 = tf_conversions.transformations.quaternion_multiply(q0, q_turn_90)
    q2 = tf_conversions.transformations.quaternion_multiply(q1, q_turn_90)
    q3 = tf_conversions.transformations.quaternion_multiply(q2, q_turn_90)

    pose1 = copy.deepcopy(pose0)
    pose1.pose.orientation = geometry_msgs.msg.Quaternion(*q1)
    pose2 = copy.deepcopy(pose0)
    pose2.pose.orientation = geometry_msgs.msg.Quaternion(*q2)
    pose3 = copy.deepcopy(pose0)
    pose3.pose.orientation = geometry_msgs.msg.Quaternion(*q3)
    
    
    poses = [pose0, pose1, pose2, pose3]

    self.cycle_through_calibration_poses(poses, "a_bot", speed=0.3)
    return 

  def cycle_through_calibration_poses(self, poses, robot_name, speed=0.3):
    rospy.loginfo("Moving all robots home.")
    self.go_to_named_pose("home_a", "a_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_c", "c_bot")
    home_pose = "home_" + robot_name[0]
    
    rospy.loginfo("============ Moving " + robot_name + " to " + poses[0].header.frame_id)
    self.go_to_pose_goal(robot_name, poses[0],speed=speed)
    for pose in poses[1:]:  
      rospy.loginfo("============ Press `Enter` to move " + robot_name + " to " + pose.header.frame_id)
      raw_input()
      self.go_to_named_pose(home_pose, robot_name)
      if rospy.is_shutdown():
        break
      self.go_to_pose_goal(robot_name, pose,speed=speed)
    
    rospy.loginfo("============ Press `Enter` to move " + robot_name + " home")
    raw_input()
    rospy.loginfo("Moving all robots home again.")
    self.go_to_named_pose("home_a", "a_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_c", "c_bot")
    return


if __name__ == '__main__':
  try:
    c = CalibrationClass()

    while True:
      rospy.loginfo("============ Calibration procedures ============ ")
      rospy.loginfo("Enter a number to check calibrations for the following things: ")
      rospy.loginfo("1: The robots")
      rospy.loginfo("2: Taskboard")
      rospy.loginfo("3: Taskboard extended fun tour")
      rospy.loginfo("4: Placement mat (for the taskboard task)")
      rospy.loginfo("5: a_bot gripper frame (rotate around EEF axis)")
      rospy.loginfo("x: Exit ")
      rospy.loginfo(" ")
      r = raw_input()
      if r == '1':
        c.check_robot_calibration()
      elif r == '2':
        c.taskboard_calibration()
      elif r == '3':
        c.taskboard_calibration_extended()
      elif r == '4':
        c.taskboard_calibration_mat()
      elif r == '5':
        c.gripper_frame_calibration_mat()
      elif r == 'x':
        break
      else:
        rospy.loginfo("Could not read: " + r)
    rospy.loginfo("============ Exiting!")

  except rospy.ROSInterruptException:
    print "Something went wrong."
