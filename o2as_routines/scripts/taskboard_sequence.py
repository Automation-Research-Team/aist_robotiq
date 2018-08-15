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

class TaskboardClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the taskboard task.
  """
  def set_up_item_parameters(self):
    # These parameters should probably be read from a csv file.
    self.item_names = ["Bearing with housing", "6 mm bearing retainer pin", "17 mm spacer for bearings", 
                      "9 mm spacer for bearings", "Rotary shaft", "4 mm round belt", 
                      "M6 Nut & Bolt", "M12 nut", "6 mm washer", 
                      "10 mm washer", "M3 set screw", "M3 bolt", 
                      "M4 bolt", "Pulley", "10 mm end cap"]
    self.item_pick_heights = [0.02, 0.02, 0.02, 
                              0.02, 0.02, 0.02, 
                              0.02, 0.02, 0.02, 
                              0.02, 0.02, 0.02,
                              0.02, 0.02, 0.02]
    self.item_place_heights = [0.04, 0.04, 0.04,
                               0.04, 0.04, 0.04, 
                               0.04, 0.04, 0.04, 
                               0.04, 0.04, 0.04, 
                               0.04, 0.04, 0.04]
    self.gripper_operation_to_use = ["outer", "inner_from_inside", "inner_from_outside", "complex_pick_from_inside", "complex_pick_from_outside"]
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    # 
    self.pick_poses = []
    self.place_poses = []
    for i in range(0,15):
      pick_pose = geometry_msgs.msg.PoseStamped()
      pick_pose.pose.orientation = downward_orientation
      pick_pose.header.frame_id = "mat_part" + str(i+1)
      pick_pose.pose.position.z = self.item_pick_heights[i]
      self.pick_poses.append(pick_pose)

      place_pose = geometry_msgs.msg.PoseStamped()
      place_pose.pose.orientation = downward_orientation
      place_pose.header.frame_id = "taskboard_part" + str(i+1)
      place_pose.pose.position.z = self.item_place_heights[i]
      self.place_poses.append(place_pose)
    



  def precision_gripper_outer_close(self):
    rospy.wait_for_service('precision_gripper_command')
    try:
        precision_gripper_client = rospy.ServiceProxy('precision_gripper_command',PrecisionGripperCommand)
        rospy.sleep(.5)
        
        request = PrecisionGripperCommandRequest()
        request.close_outer_gripper_fully = True
        
        rospy.loginfo("Closing outer gripper")
        precision_gripper_client(request)
        rospy.loginfo("Waiting for 5 seconds")
        rospy.sleep(5)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  def precision_gripper_outer_open(self):
    rospy.wait_for_service('precision_gripper_command')
    try:
        precision_gripper_client = rospy.ServiceProxy('precision_gripper_command',PrecisionGripperCommand)
        rospy.sleep(.5)
        request = PrecisionGripperCommandRequest()
        request.open_outer_gripper_fully = True
        request.close_outer_gripper_fully = False

        rospy.loginfo("Opening outer gripper")
        precision_gripper_client(request)
        rospy.loginfo("Waiting for 5 seconds")
        rospy.sleep(5)

        request.stop = True
        request.open_outer_gripper_fully = False
        request.close_outer_gripper_fully = False

        rospy.loginfo("Disabling torque (stopping the gripper)")
        precision_gripper_client(request)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


  ################ ----- Routines  
  ################ 
  ################ 
    
  def simple_taskboard_demo(self):
    self.groups["a_bot"].set_goal_tolerance(.0001) 
    self.groups["a_bot"].set_planning_time(5) 
    self.go_to_named_pose("home_c", "c_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_a", "a_bot")
    
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))

    # Perform the pick/place operations
    for i in range(0,1):
      i=3
      rospy.loginfo("=== Now targeting part number " + str(i+1) + ": " + self.item_names[i])

      self.groups["a_bot"].set_goal_tolerance(.000001) 
      self.groups["a_bot"].set_planning_time(10) 
      self.groups["a_bot"].set_num_planning_attempts(1000) 

      self.do_pick_action("a_bot", pick_poses[j], tool_name = "", do_complex_pick_from_inside=True)
      self.do_place_action("a_bot", place_poses[j], tool_name = "")
      
      
      # place_poses[j].pose.position.z += (.1)
      # self.go_to_pose_goal("a_bot", place_poses[j], speed=0.3)
      # place_poses[j].pose.position.z -= (.1)
      # self.go_to_pose_goal("a_bot", place_poses[j], speed=0.02)

      #self.go_to_named_pose("home_a", "a_bot")
    return



if __name__ == '__main__':
  try:
    c = TaskboardClass()
    c.simple_taskboard_demo()

    print "============ Done!"
  except rospy.ROSInterruptException:
    return
