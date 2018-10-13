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
from math import *

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg

import o2as_msgs
import o2as_msgs.srv


from o2as_routines.base import O2ASBaseRoutines

class TaskboardClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the taskboard task.
  """
  def __init__(self):
    super(TaskboardClass, self).__init__()
    self.set_up_item_parameters()
    
    # self.action_client.wait_for_server()
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

  def set_up_item_parameters(self):
    # These parameters should probably be read from a csv file.
    self.item_names = ["Bearing with housing", "6 mm bearing retainer pin", "17 mm spacer for bearings", 
                      "9 mm spacer for bearings", "Rotary shaft", "4 mm round belt", 
                      "M6 Nut & Bolt", "M12 nut", "6 mm washer", 
                      "10 mm washer", "M3 set screw", "M3 bolt", 
                      "M4 bolt", "Pulley", "10 mm end cap"]
    self.item_pick_heights = [0.02, 0.02, 0.042,
                              0.047, 0.072, 0.0, 
                              0.02, 0.02, -0.002, 
                              -0.002, 0.001, 0.0,
                              0.005, 0.007, 0.004]

    self.item_place_heights = [0.02, 0.0, 0.046,
                               0.046, 0.074, 0.04, 
                               0.04, 0.04, 0.0, 
                               0.0, 0.0, 0.04, 
                               0.04, 0.006, 0.0]
    self.gripper_operation_to_use = ["outer", "inner_from_inside", "inner_from_outside", "complex_pick_from_inside", "complex_pick_from_outside"]
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    self.downward_orientation_cylinder_axis_along_workspace_x = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    # 
    self.pick_poses = []
    self.place_poses = []
    for i in range(1,16):
      pick_pose = geometry_msgs.msg.PoseStamped()
      pick_pose.pose.orientation = self.downward_orientation
      if i == 11:
        pick_pose.pose.orientation = self.downward_orientation_cylinder_axis_along_workspace_x
      pick_pose.header.frame_id = "mat_part" + str(i)
      # pick_pose.pose.position.z = self.item_pick_heights[i]
      self.pick_poses.append(pick_pose)

      place_pose = geometry_msgs.msg.PoseStamped()
      place_pose.pose.orientation = self.downward_orientation
      place_pose.header.frame_id = "taskboard_part" + str(i)
      # 
      # place_pose.pose.position.z = self.item_place_heights[i]
      self.place_poses.append(place_pose)
   

  ################ ----- Routines  
  ################ 
  ################ 
      
  def belt_circle_motion(self, robot_name, speed = 0.02):
    self.toggle_collisions(collisions_on=False)
    group = self.groups[robot_name]
    rospy.loginfo("Performing belt spiral motion " + str(speed))
    rospy.loginfo("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)
    
    if number == 1:
      r_pulley=0.034
    else:
      r_pulley=0.019

    theta_offset = 90  # To adjust the starting angle
    theta_belt= 0 + theta_offset
    theta_increase=20

    start_pose = geometry_msgs.msg.PoseStamped()
    start_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    start_pose.header.frame_id = "taskboard_part6_large_pulley"
    start_pose.pose.position.z = 0.07
    start_pose.pose.position.x = cos(radians(theta_belt))*r_pulley
    start_pose.pose.position.y = sin(radians(theta_belt))*r_pulley
    self.go_to_pose_goal(robot_name, start_pose, speed=0.5, move_lin=True)

    start_pose.pose.position.z = 0
    self.go_to_pose_goal(robot_name, start_pose, speed=0.02, move_lin=True)
    
    next_pose = geometry_msgs.msg.PoseStamped()
    next_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    next_pose.header.frame_id = "taskboard_part6_large_pulley"
    while theta_belt <= 340+theta_offset and not rospy.is_shutdown():
        #By default, the Spiral_Search function will maintain contact between both mating parts at all times
         theta_belt=theta_belt+theta_increase
         x=cos(radians(theta_belt))*r_pulley
         y=sin(radians(theta_belt))*r_pulley
         next_pose.pose.position.x = x
         next_pose.pose.position.y = y
         print(theta_belt)
        #  print(radians(theta_belt))
         print(cos(radians(theta_belt)))
         print(cos(radians(theta_belt))*r_pulley)
         print(next_pose.pose.position)
         self.go_to_pose_goal(robot_name, next_pose, move_lin=True)
         rospy.sleep(0.1)
    
    self.toggle_collisions(collisions_on=True)
    # -------------
    return True
  
  
  ####

  def full_taskboard_task(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "a_bot")

    # Proposed order:
    # - Set screw 11 first, because it is affected by any movement of the mat
    # - Washers 9 and 10 early, because they are easy
    # - Spacers 3, 4 early, because they are easy and tall
    # - Pulley 14 early, because it is easy
    # 
    # - End cap 15 after washer 10
    # - Small screws 12, 13 not too late, because they may fall over, but not too early because things might be in the way
    # - Retainer pin 2 late, because we need space to pick it up and it is risky + expensive (few points, but takes time)
    # - Big nut 8 late, because it may move the mat
    # 
    # - Belt 6 after bearing 1. nut/bolt 7 anytime.
    
    # Set screw has to be first, because b_bot is right on top of it
    self.do_task_number(11) # set screw 
    self.do_task_number(2)  # Retainer pin

    self.do_task_number(3)  # Spacer small
    self.do_task_number(4)  # Spacer large

    self.do_task_number(9)  # Washer small
    self.do_task_number(10) # Washer large
    self.do_task_number(14) # Retainer pin

    self.do_task_number(1)  # Bearing
    self.do_task_number(6)  # Belt
    
    self.do_task_number(15) # end cap
    self.do_task_number(13) # M3 screw?
    self.do_task_number(12) # M4 screw?

    self.do_task_number(8)  # M10 nut
    self.do_task_number(7)  # M6 nut/bolt
    

  def tilt_up_gripper(self, speed_fast=0.1, speed_slow=0.02, screw_size=4):
    self.go_to_named_pose("tilt_ready", "a_bot")
    rospy.sleep(.2)
    before_tilt = geometry_msgs.msg.PoseStamped()
    before_tilt.header.frame_id = "a_bot_gripper_tip_link"
    before_tilt.pose.orientation.w = 1.0
    before_tilt = self.listener.transformPose("workspace_center", before_tilt)
    # before_tilt.pose.position.x += 0.25
    # before_tilt.pose.position.z += 0.3
    before_tilt_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, pi*45/180))
    before_tilt.pose.orientation = before_tilt_orientation
    self.go_to_pose_goal('a_bot', before_tilt, speed=speed_fast)

    #tilt up
    tilt_1 = copy.deepcopy(before_tilt)
    tilt_1.pose.position.z += 0.1
    tilt_1.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi*35/180, pi*45/180))
    self.go_to_pose_goal('a_bot', tilt_1, speed=speed_fast)

    #gripper slight open
    if screw_size == 4:
      self.precision_gripper_inner_open_slightly(open_range=50)
    if screw_size == 3:
      self.precision_gripper_inner_open_slightly(open_range=30)

    self.go_to_pose_goal('a_bot', before_tilt, speed=speed_fast)

    tilt_2 = copy.deepcopy(before_tilt)
    tilt_2.pose.position.z += 0.1
    tilt_2.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi*45/180, pi*45/180))
    self.go_to_pose_goal('a_bot', tilt_2, speed=speed_fast)

    self.go_to_pose_goal('a_bot', before_tilt, speed=speed_fast)

  def equip_unequip_set_screw_tool(self, equip=True):
    pick_up_set_screw_tool_pose = geometry_msgs.msg.PoseStamped()
    pick_up_set_screw_tool_pose.header.frame_id = "taskboard_set_screw_tool_link"
    pick_up_set_screw_tool_pose.pose.position.x = -.005
    pick_up_set_screw_tool_pose.pose.orientation.w = 1.0
    if equip: # Pick up tool
      taskboard.send_gripper_command(gripper="b_bot", command="open")
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.06, move_lin=True)
      taskboard.send_gripper_command(gripper="b_bot", command="close")
      print("Press enter to proceed.")
      inp = raw_input()
      pick_up_set_screw_tool_pose.pose.position.x -= .03
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.04, move_lin=True)
      pick_up_set_screw_tool_pose.pose.position.x += .03
      taskboard.go_to_named_pose("set_screw_intermediate_pose", "b_bot")
    elif not equip: # Place tool
      pick_up_set_screw_tool_pose.pose.position.x -= .01
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.04, move_lin=True)
      pick_up_set_screw_tool_pose.pose.position.x += .01
      taskboard.send_gripper_command(gripper="b_bot", command="open")
      pick_up_set_screw_tool_pose.pose.position.x -= .06
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.06, move_lin=True)
      pick_up_set_screw_tool_pose.pose.position.x += .06
      taskboard.go_to_named_pose("set_screw_intermediate_pose", "b_bot")

  def equip_unequip_belt_tool(self, equip=True):
    pick_up_set_screw_tool_pose = geometry_msgs.msg.PoseStamped()
    pick_up_set_screw_tool_pose.header.frame_id = "taskboard_set_screw_tool_link"
    pick_up_set_screw_tool_pose.pose.position.x = -.005
    pick_up_set_screw_tool_pose.pose.orientation.w = 1.0
    if equip: # Pick up tool
      taskboard.send_gripper_command(gripper="b_bot", command="open")
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.06, move_lin=True)
      taskboard.send_gripper_command(gripper="b_bot", command="close")
      print("Press enter to proceed.")
      inp = raw_input()
      pick_up_set_screw_tool_pose.pose.position.x -= .03
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.04, move_lin=True)
      pick_up_set_screw_tool_pose.pose.position.x += .03
      taskboard.go_to_named_pose("set_screw_intermediate_pose", "b_bot")
    elif not equip: # Place tool
      pick_up_set_screw_tool_pose.pose.position.x -= .01
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.04, move_lin=True)
      pick_up_set_screw_tool_pose.pose.position.x += .01
      taskboard.send_gripper_command(gripper="b_bot", command="open")
      pick_up_set_screw_tool_pose.pose.position.x -= .06
      taskboard.go_to_pose_goal("b_bot", pick_up_set_screw_tool_pose, speed=0.06, move_lin=True)
      pick_up_set_screw_tool_pose.pose.position.x += .06
      taskboard.go_to_named_pose("set_screw_intermediate_pose", "b_bot")

  def do_task_number(self, i):
    if i == 1: 
      b_bot_dx_pick = 0.0 ## MAGIC NUMBER
      b_bot_dy_pick = 0.0 ## MAGIC NUMBER
      bearing_pick_pose_b = copy.deepcopy(self.pick_poses[i-1])
      bearing_pick_pose_b.pose.position.x += b_bot_dx_pick
      bearing_pick_pose_b.pose.position.y += b_bot_dy_pick

      self.go_to_named_pose("home","a_bot")
      self.go_to_named_pose("home","b_bot")
      self.go_to_named_pose("back","c_bot")
      self.pick("b_bot",self.pick_poses[i-1],self.item_pick_heights[i-1],
                      speed_fast = 0.5, speed_slow = 0.1, gripper_command="close",
                      approach_height = 0.07)

      self.pick_poses[i-1].pose.position.z += 0.2
      self.go_to_pose_goal("b_bot", self.pick_poses[i-1], speed=0.5, move_lin=True)

      bearing_b_place_pose = copy.deepcopy(self.place_poses[i-1])  # This is part 4 ???
      bearing_b_place_pose.pose.position.x = .03
      bearing_b_place_pose.pose.position.y = .03
      bearing_b_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      bearing_b_place_pose.pose.position.z = self.item_place_heights[i-1] + .001

      bearing_b_place_pose_approach = copy.deepcopy(bearing_b_place_pose)
      bearing_b_place_pose_approach.pose.position.z = .15
      self.go_to_pose_goal("b_bot", bearing_b_place_pose_approach, speed=0.5, move_lin=True)
      self.go_to_pose_goal("b_bot", bearing_b_place_pose, speed=0.15, move_lin=True)
      self.do_linear_push("b_bot", 5, wait = True)
      self.send_gripper_command(gripper="b_bot", command="open")
      rospy.sleep(1.0)
      self.send_gripper_command(gripper="b_bot", command="close", force=1.0, velocity = .013)
      rospy.sleep(1.0)
      self.send_gripper_command(gripper="b_bot", command="open", velocity = .013)
      rospy.sleep(2.0)

      self.go_to_named_pose("home","b_bot")

      self.send_gripper_command(gripper="precision_gripper_inner", command="close")
      
      bearing_b_place_pose.pose.position.z = 0.0
      
      z_a_bot = 0.015
      self.place("a_bot", bearing_b_place_pose, z_a_bot,
                              speed_fast = 0.5, speed_slow = 0.02, gripper_command="none",
                              approach_height = 0.05, lift_up_after_place = False)
      bearing_a_place_pose_final = copy.deepcopy(self.place_poses[i-1])
      bearing_a_place_pose_final.pose.position.z = z_a_bot
      rospy.loginfo("Moving bearing to final pose")
      self.go_to_pose_goal("a_bot", bearing_a_place_pose_final, speed=0.15, move_lin=True)
      self.horizontal_spiral_motion("a_bot", .004)
      bearing_a_place_pose.pose.position.z += .05
      self.go_to_pose_goal("a_bot", bearing_a_place_pose, speed=0.15, move_lin=True)
      self.go_to_named_pose("home","a_bot")
      
      # ### Push with b_bot
      ### Pushing with b_bot only does not work. The tool needs to be grasped beforehand.
      self.go_to_pose_goal("b_bot", bearing_a_place_pose, speed=0.15, move_lin=True)
      self.send_gripper_command(gripper="b_bot", command=0.04)
      self.do_linear_push("b_bot", 20, wait = True)
      self.go_to_pose_goal("b_bot", bearing_a_place_pose, speed=0.15, move_lin=True)
      self.go_to_named_pose("home","b_bot")
      rospy.loginfo("Done")

      # TODO: Make sure the task succeeded by pushing with b_bot and plate 3

    if i == 2: #unadjusted 
      ###set the tool
      tool_pickup_pose = geometry_msgs.msg.PoseStamped()
      tool_pickup_pose.header.frame_id = "retainer_pin_insertion_tool"
      tool_pickup_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
      tool_grasped_height = 0.03
      self.pick("b_bot",tool_pickup_pose, tool_grasped_height,
                          speed_fast = 1.0, speed_slow = 0.5, gripper_command="close",
                          approach_height = 0.1)
      tool_pickup_pose_high = copy.deepcopy(tool_pickup_pose)
      tool_pickup_pose_high.pose.position.z += .2
      self.go_to_pose_goal("b_bot", tool_pickup_pose_high, speed=1.0, move_lin=True)

      tool_place_approach = copy.deepcopy(self.place_poses[i-1])
      tool_place_approach.pose.position.z += .15
      self.go_to_pose_goal("b_bot", tool_place_approach, speed=1.0, move_lin=True)
      self.place("b_bot",self.place_poses[i-1], tool_grasped_height + .001,
                      speed_fast = 1.0, speed_slow = 0.5, gripper_command="none",
                      approach_height = 0.05, lift_up_after_place = False)
      self.send_gripper_command(gripper="b_bot", command=.02, velocity = .013)
      rospy.sleep(2.0)
      self.send_gripper_command(gripper="b_bot", command="open")
      self.go_to_named_pose("home","b_bot")
      
      ### a_bot screw pickup/place routine
      pickup_pin_with_a_bot = False
      if pickup_pin_with_a_bot:
        ###pick up screw
        inclined_pick_pose = copy.deepcopy(self.pick_poses[i-1])
        inclined_pick_pose.pose.position.x += -.025
        inclined_pick_pose.pose.position.z += .01
        inclined_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/5, pi))

        self.pick("a_bot",inclined_pick_pose, 0.00,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_outside_only_inner",
                                approach_height = 0.05, special_pick = False)
        inclined_pick_pose_high = copy.deepcopy(inclined_pick_pose)
        inclined_pick_pose_high.pose.position.z += 0.2
        self.go_to_pose_goal("a_bot", inclined_pick_pose_high, speed=0.1, move_lin=True)
        
        ###place the pin
        pin_place_approach = copy.deepcopy(self.place_poses[i-1])
        pin_place_approach.pose.position.x += -.025
        pin_place_approach.pose.position.z = 0.2
        pin_place_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/5, pi))
        self.go_to_pose_goal("a_bot", pin_place_approach, speed=0.1, move_lin=True)
        
        pin_place_pose = copy.deepcopy(pin_place_approach)
        pin_place_pose.pose.position.z = 0.07
        self.go_to_pose_goal("a_bot", pin_place_pose, speed=0.1, move_lin=True)
        pin_place_pose.pose.position.x = 0

        self.send_gripper_command(gripper="precision_gripper_inner", command="open")

        pin_place_pose.pose.position.y = 0.0
        pin_place_pose.pose.position.z += 0.1
        pin_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
        self.go_to_pose_goal("a_bot", pin_place_pose, speed=0.1)
        self.go_to_named_pose("home","a_bot")
      else:
        pin_pick_approach = copy.deepcopy(self.pick_poses[i-1])
        pin_pick_approach.pose.position.z += .15

        self.go_to_pose_goal("b_bot", pin_pick_approach, speed=1.0, move_lin=True)
        self.pick("b_bot", self.pick_poses[i-1], grasp_height=0.02,
                                speed_fast = 1.0, speed_slow = 0.2, gripper_command="",
                                approach_height = 0.1, special_pick = False)
        self.go_to_pose_goal("b_bot", pin_pick_approach, speed=1.0, move_lin=True)

        self.adjust_centering(go_fast=True)

        pin_place_approach = copy.deepcopy(self.place_poses[i-1])
        pin_place_approach.pose.position.z += 0.2

        self.go_to_pose_goal("b_bot", pin_place_approach, speed=0.1, move_lin=True)
        self.place("b_bot",self.place_poses[i-1], place_height=.01+.06,
                      speed_fast = 1.0, speed_slow = 0.02, gripper_command="open",
                      approach_height = 0.1, lift_up_after_place = False)
        self.send_gripper_command(gripper="b_bot", command=.05)
        rospy.sleep(.5)
        self.go_to_pose_goal("b_bot", pin_place_approach, speed=0.1, move_lin=True)

      ### Lift tool with b_bot and do spiral motion
      b_pose = copy.deepcopy(self.place_poses[i-1])
      b_pose.pose.position.z = 0.15
      
      self.go_to_pose_goal("b_bot", b_pose, speed=1.0)
      b_pose.pose.position.z -= 0.12
      self.go_to_pose_goal("b_bot", b_pose, speed=1.0)
      self.send_gripper_command(gripper="b_bot", command="close")
      rospy.sleep(1.0)
      
      b_pose.pose.position.z += 0.01
      self.go_to_pose_goal("b_bot", b_pose, speed=0.02)
      self.horizontal_spiral_motion("b_bot", max_radius = .005, radius_increment = .003)

      b_pose.pose.position.z = 0.1
      self.go_to_pose_goal("b_bot", b_pose, speed=1.0, move_lin=True)
        
      ### Push in the pin
      b_pose.pose.position.x += 0.004
      self.go_to_pose_goal("b_bot", b_pose, speed=1.0, move_lin=True)
      self.do_linear_push("b_bot", 5, wait = True)
      b_pose.pose.position.z += 0.1
      self.go_to_pose_goal("b_bot", b_pose, speed=1.0, move_lin=True)
      
      ### Discard the tool (TODO: Joint pose)
      # b_pose.pose.position.x += 0.4
      # b_pose.pose.position.y += 0.4
      # self.go_to_pose_goal("b_bot", b_pose, speed=0.2, move_lin=True)
      self.go_to_named_pose("discard_taskboard_tool","b_bot", speed=2.0, acceleration=2.0, force_ur_script=True)
      self.send_gripper_command(gripper="b_bot", command="open")
      
      self.go_to_named_pose("home","b_bot", speed=2.0, acceleration=2.0, force_ur_script=True)

    if i == 3:   # Washer
      self.pick("a_bot",self.pick_poses[i-1],self.item_pick_heights[i-1],
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              approach_height = 0.07)
      self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")
      self.place("a_bot",self.place_poses[i-1],self.item_place_heights[i-1],
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              approach_height = 0.1, lift_up_after_place = False)
      self.horizontal_spiral_motion("a_bot", .004)
      rospy.loginfo("doing spiral motion")
    
    if i == 4:    # Washer
      self.pick("a_bot",self.pick_poses[i-1],self.item_pick_heights[i-1],
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              approach_height = 0.07)
      self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")
      self.place("a_bot",self.place_poses[i-1],self.item_place_heights[i-1],
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              approach_height = 0.1, lift_up_after_place = False)
      self.horizontal_spiral_motion("a_bot", max_radius=.004, radius_increment=.002)
    
    if i == 5:
      rospy.loginfo("Part 5 was deleted and is skipped.")
      pass

    if i == 6: # Belt
      self.go_to_named_pose("back", "c_bot")
      
      # Set the placement aid
      self.go_to_named_pose("home", "b_bot")
      belt_tool_pick_pose = geometry_msgs.msg.PoseStamped()
      belt_tool_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      belt_tool_pick_pose.header.frame_id = "belt_placement_tool"
      belt_tool_grasp_height = .11
      self.pick("b_bot", belt_tool_pick_pose, grasp_height=belt_tool_grasp_height, speed_fast = 0.5, speed_slow = 0.1, gripper_command="close",
                              approach_height = 0.05)
      self.go_to_named_pose("home", "b_bot")

      # TODO: Push on the bearing?
      
      belt_tool_place_pose = geometry_msgs.msg.PoseStamped()
      belt_tool_place_pose.header.frame_id = "taskboard_part6_small_pulley"
      belt_tool_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      belt_tool_place_pose.pose.position.y = .07
      belt_tool_place_pose.pose.position.z = belt_tool_grasp_height + .002

      belt_tool_place_pose_approach = copy.deepcopy(belt_tool_place_pose)
      belt_tool_place_pose.pose.position.z += .15

      self.move_lin("b_bot", belt_tool_place_pose_approach, speed=1.0)
      self.move_lin("b_bot", belt_tool_place_pose, speed=.3)

      self.do_linear_push("b_bot", direction="Y-", force=3)
      self.send_gripper_command("b_bot", .01)
      self.send_gripper_command("b_bot", "open")
      self.move_lin("b_bot", belt_tool_place_pose_approach, speed=1.0)

      # self.place("b_bot", belt_tool_place_pose, place_height=grasp_height, speed_fast = 0.5, speed_slow = 0.05, gripper_command="open",
      #                         approach_height = 0.03, lift_up_after_place = True)
      
      self.go_to_named_pose("taskboard_center_pose", "b_bot")

      # Pick up the belt
      belt_pick_pose = copy.deepcopy(self.pick_poses[5])
      belt_pick_pose.pose.position.y += .055
      self.pick("b_bot", belt_pick_pose, grasp_height=.002,
                      speed_fast = 0.2, speed_slow = 0.04, gripper_command="close")
      self.go_to_named_pose("taskboard_center_pose", "b_bot")

      # Place the belt
      belt_place_pose = geometry_msgs.msg.PoseStamped()
      belt_place_pose.header.frame_id = "taskboard_part6_large_pulley"
      belt_place_pose.pose.position.x = 0.0
      belt_place_pose.pose.position.y = .068
      belt_place_pose.pose.position.z = .06 + .1
      belt_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      self.go_to_pose_goal("b_bot", belt_place_pose, speed=0.3)
      belt_place_pose.pose.position.z = .06
      self.go_to_pose_goal("b_bot", belt_place_pose, speed=0.3)

      # In large pulley frame
      belt_place_pose.header.frame_id = "taskboard_part6_large_pulley"
      belt_place_pose.pose.position.x = 0.0
      belt_place_pose.pose.position.y = .01
      belt_place_pose.pose.position.z = .0085
      belt_place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      self.go_to_pose_goal("b_bot", belt_place_pose, speed=0.1)
      
      self.send_gripper_command(gripper="b_bot", command=.01)
      rospy.sleep(1)
      self.send_gripper_command(gripper="b_bot", command="open")
      belt_place_pose.pose.position.z += .02
      self.go_to_pose_goal("b_bot", belt_place_pose, speed=1.0)
      rospy.sleep(1)
      self.go_to_named_pose("back", "b_bot", wait=True)

      # Fiddle in the belt
      rospy.logwarn("Doing belt spiral motion")
      self.belt_circle_motion("a_bot")
      self.go_to_named_pose("home", "a_bot", wait=True)

      # Remove the placement aid
      self.go_to_named_pose("taskboard_center_pose", "b_bot")
      self.pick("b_bot", belt_tool_place_pose, grasp_height=belt_tool_grasp_height, speed_fast = 1.0, speed_slow = 0.1, gripper_command="close",
                              approach_height = 0.15)
      self.place("b_bot", belt_tool_place_pose, place_height=belt_tool_grasp_height+.002, speed_fast = 0.2, speed_slow = 0.03, gripper_command="open",
                              approach_height = 0.03, lift_up_after_place = True)

    if i == 7:
      # Pick up M6 screw, arrange it in the gripper, and pick it with b_bot
      partScrew = geometry_msgs.msg.PoseStamped()
      partScrew.header.frame_id = "mat_part7_1"
      partScrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/4, -pi/2))
      self.pick("a_bot",partScrew, grasp_height=.01,
                      speed_fast = 0.2, speed_slow = 0.02, gripper_command="close",
                      approach_height = 0.1)
      self.tilt_up_gripper(speed_fast=0.1, speed_slow=0.02)

      self.do_change_tool_action("b_bot", equip=True, screw_size = 6)        
      self.pick_screw_from_precision_gripper(screw_size=4, robot_name="b_bot")
      self.go_to_named_pose("home", "a_bot")
      self.go_to_named_pose("screw_ready", "b_bot")
      self.go_to_named_pose("screw_ready_back", "b_bot")

      # Pick the nut with a_bot, place it near c_bot
      self.go_to_named_pose("home", "a_bot")
      nut_pick_pose = geometry_msgs.msg.PoseStamped()
      nut_pick_pose.header.frame_id = "mat_part7_2"
      nut_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      self.pick("a_bot",nut_pick_pose, 0.0,
                                  speed_fast = 0.31, speed_slow = 0.05, gripper_command="inner_gripper_from_inside",
                                  approach_height = 0.05)
      self.go_to_named_pose("home", "a_bot")     

      nut_place_a_bot = geometry_msgs.msg.PoseStamped()
      nut_place_a_bot.header.frame_id = "workspace_center"
      nut_place_a_bot.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
      nut_place_a_bot.pose.position.x = -.40
      nut_place_a_bot.pose.position.y = -.25
      nut_place_a_bot.pose.position.z = -.015

      nut_place_c_bot = copy.deepcopy(nut_place_a_bot)
      nut_place_c_bot.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/4))

      self.place("a_bot",nut_place_a_bot,0.0,
                                  speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                  approach_height = 0.05,lift_up_after_place = True)
      self.go_to_named_pose("back", "a_bot")   

      # Pick up the nut with c_bot
      self.do_change_tool_action("c_bot", equip=True, screw_size=66)  #66 = nut tool m6
      self.pick_nut_from_table(object_pose=nut_place_c_bot,end_effector_link="c_bot_nut_tool_m6_tip_link")

      # Position b_bot screw tool (via intermediate pose?)
      # self.go_to_named_pose("taskboard_screw_tool_horizontal_approach", "b_bot")  #TODO
      rospy.logerr("Is the intermediate pose for the screw tool set up?")
      raw_input()
      screw_tool_hold = geometry_msgs.msg.PoseStamped()
      screw_tool_hold.header.frame_id = "taskboard_part7_1"
      screw_tool_hold.pose.orientation.w = 1.0
      screw_tool_hold_approach = copy.deepcopy(screw_tool_hold)
      screw_tool_hold_approach.pose.position.x = -.01
      screw_tool_hold_approach_high = copy.deepcopy(screw_tool_hold_approach)
      screw_tool_hold_approach_high.pose.position.z = +.05
      self.go_to_pose_goal("b_bot", screw_tool_hold_approach_high, speed=.1, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.go_to_pose_goal("b_bot", screw_tool_hold_approach, speed=.01, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.go_to_pose_goal("b_bot", screw_tool_hold, speed=.01, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.horizontal_spiral_motion("b_bot", .004, radius_increment = .002, spiral_axis="Y")
      self.go_to_pose_goal("b_bot", screw_tool_hold, speed=.01, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")

      # Position c_bot 
      nut_tool_hold = geometry_msgs.msg.PoseStamped()
      nut_tool_hold.header.frame_id = "taskboard_part7_2"
      nut_tool_hold.pose.orientation.w = 1.0
      nut_tool_hold_approach = copy.deepcopy(nut_tool_hold)
      nut_tool_hold_approach.pose.position.x = -.01
      nut_tool_hold_approach_high = copy.deepcopy(nut_tool_hold_approach)
      nut_tool_hold_approach_high.pose.position.z = +.05

      # self.go_to_named_pose("taskboard_nut_tool_approach", "c_bot")  #TODO
      self.go_to_pose_goal("c_bot", nut_tool_hold_approach_high, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      self.go_to_pose_goal("c_bot", nut_tool_hold_approach, speed=.01, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      
      # Fix the nut with the nut tool
      self.set_motor("nut_tool_m6", direction = "tighten", wait=False, speed = 500, duration = 15)
      self.horizontal_spiral_motion("c_bot", .004, radius_increment = .002, spiral_axis="YZ")
      self.go_to_pose_goal("c_bot", nut_tool_hold, speed=.01, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      self.horizontal_spiral_motion("c_bot", .004, radius_increment = .002, spiral_axis="YZ")

      rospy.sleep(2.0)
      self.go_to_pose_goal("c_bot", nut_tool_hold_approach, speed=.01, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      self.go_to_pose_goal("c_bot", nut_tool_hold_approach_high, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
      self.go_to_named_pose("home", "c_bot")
      self.go_to_named_pose("tool_pick_ready", "c_bot")

      # Go home with b_bot and unequip the tool
      self.set_suction("screw_tool_m6", False, False)
      self.go_to_pose_goal("b_bot", screw_tool_hold_approach, speed=.01, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.go_to_pose_goal("b_bot", screw_tool_hold_approach_high, speed=.02, move_lin = True, end_effector_link="b_bot_screw_tool_m6_tip_link")
      self.go_to_named_pose("screw_ready", "b_bot")

    if i == 8:  
      #pick up the tool
      tool_pose = geometry_msgs.msg.PoseStamped()
      tool_pose.header.frame_id = "M10nut_tool"
      tool_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
      tool_grasped_height = 0.042

      ## MAGIC NUMBERS!!
      b_bot_dx_pick = 0.01
      b_bot_dy_pick = 0.03
      # b_bot_dx_place = 0.0
      b_bot_dy_place = 0.005

      pick_pose_low = copy.deepcopy(self.pick_poses[i-1])
      pick_pose_low.pose.position.x += b_bot_dx_pick
      pick_pose_low.pose.position.y += b_bot_dy_pick
      pick_pose_low.pose.position.z = self.item_pick_heights[i-1] + tool_grasped_height + .01
      pick_pose_high = copy.deepcopy(pick_pose_low)
      pick_pose_high.pose.position.z += .15

      place_pose_low = copy.deepcopy(self.place_poses[i-1])
      place_pose_low.pose.position.y += b_bot_dy_place
      place_pose_low.pose.position.z = .02 + tool_grasped_height + .01
      place_pose_high = copy.deepcopy(place_pose_low)
      place_pose_high.pose.position.z += .1
      
      self.pick("b_bot",tool_pose, tool_grasped_height,
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="close",
                              approach_height = 0.1)

      # Push into the nut to pick it up
      self.go_to_pose_goal("b_bot", pick_pose_low, speed=0.05, move_lin=True)
      self.do_linear_push("b_bot", 10, wait = True)
      self.horizontal_spiral_motion("b_bot", max_radius = .006, radius_increment = .01)
      self.do_linear_push("b_bot", 40, wait = True)
      rospy.sleep(2.0)

      self.go_to_pose_goal("b_bot", pick_pose_high, speed=0.05, move_lin=True)
      #place and fasten
      self.go_to_pose_goal("b_bot", place_pose_low, speed=0.05, move_lin=True)        

      self.do_nut_fasten_action("m10_nut", wait = False)
      self.do_linear_push("b_bot", 10, wait = True)
      rospy.sleep(4.0)
      self.do_linear_push("b_bot", 40, wait = True)
      rospy.sleep(8.0)

      # Go back up
      self.go_to_pose_goal("b_bot", place_pose_high, speed=0.05, move_lin=True)
      #return the tool
      self.place("b_bot",tool_pose,place_height=0.013 + tool_grasped_height,
                              speed_fast = 0.2, speed_slow = 0.07, gripper_command="open",
                              approach_height = 0.2, lift_up_after_place = True)

    if i in [9, 10]:
      self.pick("a_bot",self.pick_poses[i-1],self.item_pick_heights[i-1], approach_height = 0.03,
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")
      self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")
      self.place("a_bot",self.place_poses[i-1],self.item_place_heights[i-1], approach_height = 0.03,
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              lift_up_after_place = False)
      if i == 9:
        self.horizontal_spiral_motion("a_bot", .0025)
      if i == 10:
        self.horizontal_spiral_motion("a_bot", .006, radius_increment=0.0025)
      self.go_to_named_pose("home", "a_bot")

    
    if i == 11:      #set screw
      approach_insert_pose_b = geometry_msgs.msg.PoseStamped()
      approach_insert_pose_b.header.frame_id = "taskboard_part11"
      approach_insert_pose_b.pose.position.z = .035
      approach_insert_pose_b.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi*160/180, pi/2, 0))
      
      insert_pose_b = copy.deepcopy(approach_insert_pose_b)
      insert_pose_b.pose.position.z = 0.002

      # TODO: Change to just moving downwards and then turning on the tool
      ### Move to insertion point
      self.go_to_named_pose("set_screw_insert_pose", "b_bot")
      self.go_to_pose_goal("b_bot", approach_insert_pose_b, speed=0.04, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)
      self.set_motor("set_screw_tool", "tighten", duration = 20.0)

      ### Turn on motor, do spiral motions while descending
      self.go_to_pose_goal("b_bot", insert_pose_b, speed=0.02, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)
      self.horizontal_spiral_motion("b_bot", .003, spiral_axis="Y", radius_increment = .001)
      insert_pose_b.pose.position.z -= .002
      self.go_to_pose_goal("b_bot", insert_pose_b, speed=0.02, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)
      self.horizontal_spiral_motion("b_bot", .003, spiral_axis="Y", radius_increment = .001)
      self.go_to_pose_goal("b_bot", insert_pose_b, speed=0.01, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)
      rospy.sleep(5.0)
      
      ### Go up, drop the tool
      self.go_to_pose_goal("b_bot", approach_insert_pose_b, speed=0.02, end_effector_link="b_bot_set_screw_tool_tip_link", move_lin=True)
      self.go_to_named_pose("set_screw_insert_pose", "b_bot")
      self.go_to_named_pose("set_screw_intermediate_pose", "b_bot")
      
      # Place tool
      self.equip_unequip_set_screw_tool(equip=False)
    

    if i in [12, 13]:      # M3, M4 screw
      if i == 12:
        screw_size = 3
      elif i == 13:
        screw_size = 4

      self.go_to_named_pose("home", "a_bot")
      self.go_to_named_pose("home", "b_bot")
      self.go_to_named_pose("back", "c_bot")
      screw_pick_pose = copy.deepcopy(self.pick_poses[i-1])
      screw_pick_pose.pose.position.y += .005
      screw_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, pi*45/180, pi*90/180))
      self.pick("a_bot",screw_pick_pose,0.001,
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_outside_only_inner",
                              approach_height = 0.05, special_pick = False)    
      
      # If we decide to use the feeder, there is self.place_screw_in_feeder(screw_size) and self.pick_screw_from_feeder(screw_size)

      ###arrange M4 screw
      self.tilt_up_gripper(speed_fast=0.1, speed_slow=0.02)

      #pick up the screw tool
      self.go_to_named_pose("back", "c_bot")
      self.do_change_tool_action("b_bot", equip=True, screw_size = screw_size)
      self.go_to_named_pose("screw_ready_back", "b_bot")
      
      #pick up the screw from a_bot
      self.pick_screw_from_precision_gripper(screw_size=screw_size, robot_name="b_bot")
      self.go_to_named_pose("screw_ready", "b_bot")
      self.go_to_named_pose("home", "a_bot")

      #screw on the cap
      screw_approach = copy.deepcopy(self.place_poses[i-1])
      point_in_taskboard_frame = self.listener.transformPose("taskboard", screw_approach).pose.position
      screw_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi*135/180, 0, 0))
      
      screw_approach.pose.position.x -= 0.03
      self.go_to_pose_goal("b_bot", screw_approach, speed=0.08, end_effector_link="b_bot_screw_tool_m" + str(screw_size) + "_tip_link", move_lin=True)

      screw_pose = copy.deepcopy(screw_approach)
      screw_pose.pose.position.x = 0.001
      self.do_screw_action("b_bot", screw_pose, screw_height = 0.01, screw_size = screw_size)

      self.go_to_pose_goal("b_bot", screw_approach, speed=0.05, end_effector_link="b_bot_screw_tool_m" + str(screw_size) + "_tip_link", move_lin=True)

      self.go_to_named_pose("screw_ready", "b_bot")
      self.do_change_tool_action("b_bot", equip=False, screw_size = screw_size)

    if i == 14:
      self.pick("a_bot",self.pick_poses[i-1],self.item_pick_heights[i-1], approach_height = 0.05,
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")
      self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")

      self.place("a_bot",self.place_poses[i-1],self.item_place_heights[i-1], approach_height = 0.05,
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              lift_up_after_place = False)
      self.horizontal_spiral_motion("a_bot", .002, radius_increment = .0005)

      self.send_gripper_command(gripper="c_bot", command=0.02)
      pose = copy.deepcopy(self.place_poses[i-1])
      pose.pose.position.z += .05
      self.go_to_pose_goal("a_bot", pose, speed=0.02, move_lin=True)
      self.go_to_named_pose("home", "a_bot")
      
      # TODO: Try pushing with the a_bot's open inner gripper (it would save time)
      # Push down with the c_bot in case it is blocked
      self.go_to_named_pose("home", "c_bot")
      # TODO: Turn the pose around by 180 degrees to speed up the motion
      self.place("c_bot",self.place_poses[i-1],self.item_place_heights[i-1] - .01, approach_height = 0.02,
                      speed_fast = 0.2, speed_slow = 0.05, gripper_command="none",
                      lift_up_after_place = True)
      self.go_to_named_pose("home", "c_bot")

    if i == 15: 

      pick_with_a_bot = True
      if pick_with_a_bot:
        self.go_to_named_pose("home", "b_bot")
        self.go_to_named_pose("home", "a_bot")

        # Hold with b_bot
        b_pick_pose = copy.deepcopy(self.pick_poses[i-1])
        b_pick_pose.pose.position.z = .01
        b_pick_pose.pose.position.y -= .01
        b_pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi*135/180, pi/2, -pi/2))
        b_pick_approach = copy.deepcopy(b_pick_pose)
        b_pick_approach.pose.position.z += .02
        b_pick_retreat = copy.deepcopy(b_pick_approach)
        b_pick_retreat.pose.position.z += .25

        # self.go_to_pose_goal("b_bot", b_pick_retreat, speed=0.2, move_lin=True)
        # print("b_bot orientation?")
        # raw_input()
        # if rospy.is_shutdown():
        #   return

        self.send_gripper_command("b_bot", .025)
        joints = [1.5759320259094238, -1.212271515523092, 2.1332263946533203, -3.2381704489337366, -1.5791609922992151, -0.013281170521871388]
        self.move_joints("b_bot", joints, speed=.05)

        print("b_bot arrived?")
        raw_input()
        if rospy.is_shutdown():
          return

        self.go_to_pose_goal("b_bot", b_pick_retreat, speed=0.2, move_lin=True)
        self.go_to_pose_goal("b_bot", b_pick_approach, speed=0.2, move_lin=True)
        self.go_to_pose_goal("b_bot", b_pick_pose, speed=0.01, move_lin=True)
        self.send_gripper_command("b_bot", "close")

        print("b_bot grasped the cap?")
        raw_input()
        if rospy.is_shutdown():
          return

        # Pick with a_bot
        pick_pose = copy.deepcopy(self.pick_poses[i-1])
        pick_pose.pose.position.z = self.item_pick_heights[i-1]
        pick_approach = copy.deepcopy(pick_pose)
        pick_approach.pose.position.z += .02
        pick_approach = copy.deepcopy(pick_approach)

        self.send_gripper_command("a_bot", "close")
        self.go_to_pose_goal("a_bot", pick_approach, speed=0.5, move_lin=True)
        self.go_to_pose_goal("a_bot", pick_pose, speed=0.02, move_lin=True)
        self.horizontal_spiral_motion("a_bot", .002, radius_increment = .001)
        print("Is it in?")
        raw_input()
        if rospy.is_shutdown():
          return

        self.send_gripper_command("a_bot", "open")
        self.go_to_pose_goal("a_bot", pick_approach, speed=0.5, move_lin=True)

        self.place("a_bot", self.place_poses[i-1],self.item_place_heights[i-1],
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05, lift_up_after_place = False)

        a_bot_retreat = copy.deepcopy(self.place_poses[i-1])
        a_bot_retreat.pose.position.z += 0.03
        self.go_to_pose_goal("a_bot", a_bot_retreat, speed=0.01, move_lin= True)
        self.go_to_named_pose("home", "a_bot")

        # Push with b_bot
        self.send_gripper_command(gripper="b_bot", command="close")
        b_bot_push_pose = copy.deepcopy(self.place_poses[i-1])
        b_bot_push_pose.pose.position.z += 0.015
        self.go_to_pose_goal("b_bot", b_bot_push_pose, speed=0.2)
        self.do_linear_push("b_bot", 3, wait = True)
        self.go_to_pose_goal("b_bot", b_bot_push_pose, speed=0.03, move_lin= True)
        self.go_to_named_pose("home", "b_bot")
      else:
        self.go_to_named_pose("home", "a_bot")
        self.go_to_named_pose("taskboard_center_pose", "b_bot")

        pick_pose = copy.deepcopy(self.pick_poses[i-1])
        pick_pose.pose.position.z = self.item_pick_heights[i-1]+.003
        pick_pose_rotated = copy.deepcopy(pick_pose)
        pick_pose_rotated.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
        pick_approach = copy.deepcopy(pick_pose)
        pick_approach.pose.position.z += .02
        pick_retreat = copy.deepcopy(pick_approach)
        pick_retreat.pose.position.z += .15

        self.send_gripper_command("b_bot", .02)
        self.go_to_pose_goal("b_bot", pick_approach, speed=0.5, move_lin=True)
        self.go_to_pose_goal("b_bot", pick_pose, speed=0.02, move_lin=True)

        # Center the piece and grasp it
        self.send_gripper_command("b_bot", .009, force=40, velocity=.013)  # Close to center
        rospy.sleep(1.0)
        self.send_gripper_command("b_bot", .02, velocity=.013)
        rospy.sleep(1.0)
        self.go_to_pose_goal("b_bot", pick_pose_rotated, speed=1.0, move_lin=True)
        self.send_gripper_command("b_bot", .009, force=40, velocity=.013)  # Close to center
        rospy.sleep(1.0)
        self.send_gripper_command("b_bot", .02, velocity=.013)
        rospy.sleep(1.0)
        self.go_to_pose_goal("b_bot", pick_pose, speed=1.0, move_lin=True)
        self.send_gripper_command("b_bot", .005, force=40, velocity=.013)
        rospy.sleep(1.0)

        self.go_to_pose_goal("b_bot", pick_approach, speed=0.5, move_lin=True)

        self.place("b_bot", self.place_poses[i-1],self.item_place_heights[i-1],
                                speed_fast = 0.2, speed_slow = 0.01, gripper_command="none",
                                approach_height = 0.03, lift_up_after_place = False)
        
        self.do_linear_push("b_bot", 4, wait = True)
        self.horizontal_spiral_motion("b_bot", .001)
        self.do_linear_push("b_bot", 4, wait = True)
        self.send_gripper_command("b_bot", .02, velocity=.013)
        rospy.sleep(1.0)
                                
        b_bot_retreat = copy.deepcopy(self.place_poses[i-1])
        b_bot_retreat.pose.position.z += 0.03
        self.go_to_pose_goal("b_bot", b_bot_retreat, speed=0.01, move_lin= True)

        # Push with b_bot
        self.send_gripper_command(gripper="b_bot", command="close")
        self.do_linear_push("b_bot", 3, wait = True)
        self.go_to_pose_goal("b_bot", b_bot_retreat, speed=0.03, move_lin= True)
        self.go_to_named_pose("home", "b_bot")
    

if __name__ == '__main__':
  try:
    taskboard = TaskboardClass()
    taskboard.set_up_item_parameters()

    i = 1
    while(i):
      rospy.loginfo("Enter 11, 12, 121 to equip/unequip/discard set_screw tool")
      rospy.loginfo("Enter 13, 14, 141 to equip/unequip/discard nut_tool_m10")
      rospy.loginfo("Enter 15, 16 to equip/unequip belt placement tool")
      rospy.loginfo("Enter 17, 18, 181 to equip/unequip/discard retainer pin guide tool")
      rospy.loginfo("Enter 191, 192 to equip/unequip m4 screw tool")
      rospy.loginfo("Enter 2 to move robots to home")
      rospy.loginfo("Enter 3 to do belt circle motion with a_bot")
      rospy.loginfo("Enter 4 to do m4 screw handover with b_bot")
      rospy.loginfo("Enter 40 to do a spiral motion with a_bot")
      rospy.loginfo("Enter 41 to do the belt circle motion with a_bot (this will not move to the pulley)")
      rospy.loginfo("Enter 82 to pick part 4 and tilt gripper up")
      rospy.loginfo("Enter 91, 92,... 915 to perform part 1, 2,... 15")
      rospy.loginfo("Enter start to start the task")
      rospy.loginfo("Enter x to exit")
      i = raw_input()

      if i == "start":
        taskboard.full_taskboard_task()
      if i == "11":
        taskboard.equip_unequip_set_screw_tool(equip=True)
      if i == "12":
        taskboard.equip_unequip_set_screw_tool(equip=False)
      if i == "191":
        taskboard.do_change_tool_action("b_bot", equip=True, screw_size = 4)
      if i == "192":
        taskboard.do_change_tool_action("b_bot", equip=False, screw_size = 4)
      if i == "2":
        taskboard.go_to_named_pose("home","a_bot")
        taskboard.go_to_named_pose("home","b_bot")
        taskboard.go_to_named_pose("back","c_bot")
      if i == "4":
        taskboard.pick_screw_from_precision_gripper(screw_size=4, robot_name="b_bot")
      if i == "50":
        taskboard.go_to_pose_goal("a_bot",  taskboard.pick_poses[0], "b_bot", taskboard.pick_poses[2])
      if i == "40":
        taskboard.horizontal_spiral_motion("a_bot", .05)
      if i == "41":
        taskboard.belt_circle_motion("a_bot")
      if i == "82":
        taskboard.pick("a_bot",taskboard.pick_poses[3],taskboard.item_pick_heights[3]-0.026-0.06,
                                 speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_outside_only_inner",
                                 approach_height = 0.1, special_pick = True)
        taskboard.tilt_up_gripper()

      if i in ["91","92","93","94","95","96","97","98","99","910","911","912","913","914","915"]:
        taskboard.do_task_number(int(i[1:]))
      if i == "x":
        break
      

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass

