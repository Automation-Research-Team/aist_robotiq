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
from o2as_msgs.srv import *
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
    self.item_pick_heights = [0.02, 0.02, 0.025, 
                              0.025, 0.02, 0.02, 
                              0.02, 0.02, -0.035, 
                              -0.035, 0.02, 0.02,
                              0.02, -0.025, -0.02]

    self.item_place_heights = [0.04, 0.04, 0.035,
                               0.035, 0.04, 0.04, 
                               0.04, 0.04, -0.025, 
                               -0.025, 0.04, 0.04, 
                               0.04, -0.015, -0.01]
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
      # 
      place_pose.pose.position.z = self.item_pick_heights[i]+.01
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

  def precision_gripper_inner_close(self):
    rospy.wait_for_service('precision_gripper_command')
    try:
        precision_gripper_client = rospy.ServiceProxy('precision_gripper_command',PrecisionGripperCommand)
        rospy.sleep(.5)
        
        request = PrecisionGripperCommandRequest()
        request.close_inner_gripper_fully = True
        
        rospy.loginfo("Closing outer gripper")
        precision_gripper_client(request)
        rospy.loginfo("Waiting for 5 seconds")
        rospy.sleep(5)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  def precision_gripper_inner_open(self):
    rospy.wait_for_service('precision_gripper_command')
    try:
        precision_gripper_client = rospy.ServiceProxy('precision_gripper_command',PrecisionGripperCommand)
        rospy.sleep(.5)
        request = PrecisionGripperCommandRequest()
        request.open_inner_gripper_fully = True
        request.close_inner_gripper_fully = False

        rospy.loginfo("Opening outer gripper")
        precision_gripper_client(request)
        rospy.loginfo("Waiting for 5 seconds")
        rospy.sleep(5)

        request.stop = True
        request.open_inner_gripper_fully = False
        request.close_inner_gripper_fully = False

        rospy.loginfo("Disabling torque (stopping the gripper)")
        precision_gripper_client(request)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  # def 
  # calib_pose = geometry_msgs.msg.PoseStamped()
  #   #calib_pose.header.frame_id = "taskboard_part4"
  #   calib_pose.header.frame_id = "mat_part4"
  #   calib_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
  #   rospy.loginfo(calib_pose.pose.orientation)
  #   calib_pose.pose.position.x = 0
  #   calib_pose.pose.position.z = 0.15

  #   self.go_to_named_pose("home_a", "a_bot")
  #   self.go_to_named_pose("home_b", "b_bot")
  #   self.go_to_named_pose("home_c", "c_bot")

  #   self.go_to_pose_goal("a_bot", calib_pose, speed=0.3)
  #   print "============ Press `Enter` to move a_bot ..."
  #   raw_input()
  #   calib_pose.pose.position.z -= (.12)
  #   self.go_to_pose_goal("a_bot", calib_pose, speed=0.02)

  #   print "============ Press `Enter` to move a_bot ..."
  #   raw_input()
  #   calib_pose.pose.position.z += (.12)
  #   self.go_to_pose_goal("a_bot", calib_pose, speed=0.02)
  #   print "============ Press `Enter` to move ..."
  #   raw_input()
  #   self.go_to_named_pose("home_a", "a_bot",speed=0.3)
    

  ################ ----- Routines  
  ################ 
  ################ 
  def pick(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command):
    #initial gripper_setup
    object_pose.pose.position.z = 0.16
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_close() 
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner":
      self.precision_gripper_inner_close()

    rospy.loginfo("debug1")

    object_pose.pose.position.z = grasp_height

    rospy.loginfo(grasp_height)

    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True)

    W = raw_input("waiting for the gripper")
    #gripper close
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_open()
      self.precision_gripper_outer_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_close()
      self.precision_gripper_outer_close()
    elif gripper_command=="easy_pick_only_inner":
      self.precision_gripper_inner_close()

    object_pose.pose.position.z = (.16)
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

  def place(self,robotname,object_pose,place_height,speed_fast,speed_slow,gripper_command):
    object_pose.pose.position.z = 0.16
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

    object_pose.pose.position.z = place_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True)

    print "============ Stopping at the placement height. Press `Enter` to keep moving moving the robot ..."
    raw_input()

    #gripper open
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner":
      self.precision_gripper_inner_open()

    object_pose.pose.position.z = (.16)
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)  

    

  def full_taskboard_task(self):
    self.groups["a_bot"].set_goal_tolerance(.0001) 
    self.groups["a_bot"].set_planning_time(5) 
    self.groups["a_bot"].set_num_planning_attempts(1000) 
    self.go_to_named_pose("home_c", "c_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_a", "a_bot")

    for i in range(0,15):
      rospy.loginfo("=== Now targeting part number " + str(i+1) + ": " + self.item_names[i])

      # peg-in-hole with complex_pick_from_inside
      if i in [2, 13]:
        self.pick("a_bot",self.pick_poses[i],self.item_pick_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_inside")
        self.place("a_bot",self.place_poses[i],self.item_place_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_inside")
      
      #peg-in-hole with complex_pick_from_outside
      if i == 3:
        self.pick("a_bot",self.pick_poses[i],self.item_pick_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_outside")
        self.place("a_bot",self.place_poses[i],self.item_place_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_outside")

      #peg-in-hole with easy pick_only_inner(washers)
      if i in [8, 9, 14]:
        self.pick("a_bot",self.pick_poses[i],self.item_pick_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")
        self.place("a_bot",self.place_poses[i],self.item_place_heights[i],
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")

      # This requires a regrasp (bearing)
      if i == 0:
        rospy.logwarn("This part is skipped because it requires a regrasp")
        pass
      
      # Requires a regrasp (pin)
      if i == 1:
        rospy.logwarn("This part is skipped because it requires a regrasp")
        pass

      # Screwing
      # if i in [4, 7, 10, 11, 12]:
      #   self.pick("a_bot",self.pick_poses[i],self.item_pick_heights[i], speed_fast = 0.2, speed_slow = 0.02)
      #   self.place("a_bot",self.place_poses[i],self.item_place_heights[i], speed_fast = 0.2, speed_slow = 0.02)
      
      # Requires multiple robots (Bolt and nut)
      if i == 6:
        rospy.logwarn("This part is skipped because it requires multiple robots interacting")
        pass

      # Requires special strategy (Belt)
      if i == 5:
        rospy.logwarn("This part is skipped because it requires a special strategy")
        pass


      self.go_to_named_pose("home_c", "c_bot")
      self.go_to_named_pose("home_b", "b_bot")
      self.go_to_named_pose("home_a", "a_bot")

  def taskboard_manual_testing(self):
    self.groups["a_bot"].set_goal_tolerance(.0001) 
    self.groups["a_bot"].set_planning_time(5) 
    self.go_to_named_pose("home_c", "c_bot")
    self.go_to_named_pose("home_b", "b_bot")
    self.go_to_named_pose("home_a", "a_bot")
    
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))

    # Perform the pick/place operations
    i = raw_input("the number of the part")
    rospy.loginfo("=== Now targeting part number " + str(i+1) + ": " + self.item_names[i])
    
    self.groups["a_bot"].set_goal_tolerance(.000001) 
    self.groups["a_bot"].set_planning_time(10) 
    self.groups["a_bot"].set_num_planning_attempts(1000) 
    if i == 3:  
      self.do_pick_action("a_bot", pick_poses[i], tool_name = "", do_complex_pick_from_inside=True)
      self.do_place_action("a_bot", place_poses[i], tool_name = "")
      pass
    return


if __name__ == '__main__':
  try:
    taskboard = TaskboardClass()
    taskboard.set_up_item_parameters()
    #taskboard.full_taskboard_task()
    
    # 3,14        complex_pick_from_inside
    # 4           complex_pick_from_outside
    # 9, 10, 15   easy_pick_only_inner
    taskboard.groups["a_bot"].set_goal_tolerance(.0001) 
    taskboard.groups["a_bot"].set_planning_time(5) 
    taskboard.groups["a_bot"].set_num_planning_attempts(1000)
    taskboard.go_to_named_pose("home_c", "c_bot")
    taskboard.go_to_named_pose("home_b", "b_bot")
    taskboard.go_to_named_pose("home_a", "a_bot")

    i = raw_input("the number of the part")
    i =int(i)
    while(i):
      #14 is not adjusted yet
      if i in [3]:
        taskboard.pick("a_bot",taskboard.pick_poses[i-1],taskboard.item_pick_heights[i-1],speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_inside")
        taskboard.place("a_bot",taskboard.place_poses[i-1],taskboard.item_place_heights[i-1],speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_inside")
      if i == 4:
        taskboard.pick("a_bot",taskboard.pick_poses[i-1],taskboard.item_pick_heights[i-1],speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_outside")
        taskboard.place("a_bot",taskboard.place_poses[i-1],taskboard.item_place_heights[i-1],speed_fast = 0.2, speed_slow = 0.02, gripper_command="complex_pick_from_outside")
  
      #15 is not adjusted yet  
      if i in [9, 10]:
        taskboard.pick("a_bot",taskboard.pick_poses[i-1],taskboard.item_pick_heights[i-1],speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")
        taskboard.pick("a_bot",taskboard.place_poses[i-1],taskboard.item_place_heights[i-1],speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")
      #taskboard.place("a_bot",taskboard.place_poses[i-1],taskboard.item_place_heights[i-1],speed_fast = 0.2, speed_slow = 0.02)
      taskboard.go_to_named_pose("home_c", "c_bot")
      taskboard.go_to_named_pose("home_b", "b_bot")
      taskboard.go_to_named_pose("home_a", "a_bot")
      i = raw_input("the number of the part")
      i =int(i)
    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
