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
  This routine checks the robot calibration the robots by moving them to
  objects defined in the scene.
  """
  
  def do_calibration(self):
    calib_pose = geometry_msgs.msg.PoseStamped()
    calib_pose.header.frame_id = "workspace_center"
    calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    calib_pose.pose.position.x = -0.15
    calib_pose.pose.position.z = 0.05

    self.go_to_named_pose("home_a", "a_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_c", "c_bot")

    print "============ Press `Enter` to move a_bot to calibration position ..."
    raw_input()
    self.go_to_pose_goal("a_bot", calib_pose)

    print "============ Press `Enter` to move b_bot to calibration position ..."
    raw_input()
    self.go_to_named_pose("home_a", "a_bot")
    # self.go_to_pose_goal("b_bot", calib_pose)

    # print "============ Press `Enter` to move c_bot to calibration position ..."
    # raw_input()
    # self.go_to_named_pose("home_b", "b_bot")
    # self.go_to_pose_goal("c_bot", calib_pose)

    # print "============ Press `Enter` to move c_bot home ..."
    # raw_input()
    # self.go_to_named_pose("home_c", "c_bot")

    return
  
  def taskboard_calibration(self):
    speed=0.3

    self.go_to_named_pose("home_a", "a_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_c", "c_bot")

    calib_pose = geometry_msgs.msg.PoseStamped()
    calib_pose.header.frame_id = "taskboard_corner2"
    calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    calib_pose.pose.position.x = 0
    calib_pose.pose.position.z = 0.03

    self.go_to_pose_goal("a_bot", calib_pose,speed=speed)

    print "============ Press `Enter` to move b_bot to calibration position ..."
    raw_input()
    self.go_to_named_pose("home_a", "a_bot",speed=speed)

    calib_pose = geometry_msgs.msg.PoseStamped()
    calib_pose.header.frame_id = "taskboard_corner3"
    calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    calib_pose.pose.position.x = 0
    calib_pose.pose.position.z = 0.03

    self.go_to_pose_goal("a_bot", calib_pose,speed=speed)

    print "============ Press `Enter` to move b_bot to calibration position ..."
    raw_input()
    self.go_to_named_pose("home_a", "a_bot",speed=speed)

    calib_pose = geometry_msgs.msg.PoseStamped()
    calib_pose.header.frame_id = "taskboard_part4"
    calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    calib_pose.pose.position.x = 0
    calib_pose.pose.position.z = 0.03

    self.go_to_pose_goal("a_bot", calib_pose,speed=speed)

    print "============ Press `Enter` to move b_bot to calibration position ..."
    raw_input()
    self.go_to_named_pose("home_a", "a_bot",speed=speed)

    # self.go_to_pose_goal("b_bot", calib_pose)

    # print "============ Press `Enter` to move c_bot to calibration position ..."
    # raw_input()
    # self.go_to_named_pose("home_b", "b_bot")
    # self.go_to_pose_goal("c_bot", calib_pose)

    # print "============ Press `Enter` to move c_bot home ..."
    # raw_input()
    # self.go_to_named_pose("home_c", "c_bot")

    return

  def taskboard_calibration_mat(self):
    speed=0.3

    rospy.loginfo("Moving all robots home.")
    self.go_to_named_pose("home_a", "a_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_c", "c_bot")

    calib_pose = geometry_msgs.msg.PoseStamped()
    calib_pose.header.frame_id = "mat_corner2"
    calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    calib_pose.pose.position.x = 0
    calib_pose.pose.position.z = .05

    rospy.loginfo("Moving a_bot to " + calib_pose.header.frame_id)
    self.go_to_pose_goal("a_bot", calib_pose,speed=speed)


    calib_pose = geometry_msgs.msg.PoseStamped()
    calib_pose.header.frame_id = "mat_corner3"
    calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    calib_pose.pose.position.x = 0
    calib_pose.pose.position.z = .05

    rospy.loginfo("============ Press `Enter` to move a_bot to " + calib_pose.header.frame_id)
    raw_input()
    self.go_to_named_pose("home_a", "a_bot",speed=speed)
    self.go_to_pose_goal("a_bot", calib_pose,speed=speed)


    calib_pose = geometry_msgs.msg.PoseStamped()
    calib_pose.header.frame_id = "mat_part3"
    calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    calib_pose.pose.position.x = 0
    calib_pose.pose.position.z += 0.01
    rospy.loginfo("============ Press `Enter` to move a_bot to " + calib_pose.header.frame_id)
    raw_input()
    self.go_to_named_pose("home_a", "a_bot",speed=speed)
    self.go_to_pose_goal("a_bot", calib_pose,speed=speed)

    rospy.loginfo("============ Press `Enter` to move a_bot home.")
    raw_input()
    self.go_to_named_pose("home_a", "a_bot",speed=speed)
    return


if __name__ == '__main__':
  try:
    c = CalibrationClass()
    c.taskboard_calibration_mat()
    print "============ Calibration complete!"
  except rospy.ROSInterruptException:
    print "Something went wrong."
