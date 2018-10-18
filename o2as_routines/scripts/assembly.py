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
import tf
from math import pi
import math

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg


from o2as_routines.base import O2ASBaseRoutines

class AssemblyClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the taskboard task.
  """
  def __init__(self):
    super(AssemblyClass, self).__init__()
    self.set_up_item_parameters()
    
    # self.action_client.wait_for_server()
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

    # MAGIC NUMBERS!
    # This list is not exhaustive, but it's a start
    self.idler_pin_handover_offset_y = 0.0
    self.idler_pin_handover_offset_z = 0.0

    # Initialize debug monitor
    self.start_task_timer()
    self.log_to_debug_monitor(text="Init", category="task")
    self.log_to_debug_monitor(text="Init", category="subtask")
    self.log_to_debug_monitor(text="Init", category="operation")

  def set_up_item_parameters(self):
    # TODO: Publish the items to the scene, or do something equivalent. 
    self.item_names = []
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 
    

  ################ ----- Routines  
  ################ 
  ################ 
  def pick_joshua(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command="", approach_height = 0.03, end_effector_link=""):
    #initial gripper_setup
    rospy.loginfo("Going above object to pick")
    object_pose.pose.position.z = approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast,end_effector_link=end_effector_link, move_lin=True)

    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
      self.precision_gripper_inner_close()
    elif gripper_command=="easy_pick_outside_only_inner" or gripper_command=="inner_gripper_from_outside":
      self.precision_gripper_inner_open()
    elif gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="open")

    rospy.loginfo("Moving down to object")
    object_pose.pose.position.z = grasp_height
    rospy.loginfo(grasp_height)
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True,end_effector_link=end_effector_link, move_lin=True)

    # W = raw_input("waiting for the gripper")
    #gripper open
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
    elif gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="close")
      
    rospy.sleep(1)
    rospy.loginfo("Going back up")
    object_pose.pose.position.z = (approach_height)
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast,end_effector_link=end_effector_link, move_lin=True)

######

  def place_joshua(self,robotname, object_pose, place_height, speed_fast, speed_slow, gripper_command="", approach_height = 0.05, approach_axis="z" ,lift_up_after_place = True):
    rospy.loginfo("Going above place target")
    if approach_axis=="z":
      object_pose.pose.position.z = approach_height
    elif approach_axis=="y":
      object_pose.pose.position.y = approach_height
    elif approach_axis=="x":
      object_pose.pose.position.x = approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

    rospy.loginfo("Moving to place target")
    if approach_axis=="z":
      object_pose.pose.position.z = place_height
    elif approach_axis=="y":
      object_pose.pose.position.y = place_height
    elif approach_axis=="x":
      object_pose.pose.position.x = place_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True)

    #gripper open
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
      self.precision_gripper_inner_close()
    elif gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="open")
    
    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      if approach_axis=="z":
        object_pose.pose.position.z = approach_height
      elif approach_axis=="y":
        object_pose.pose.position.y = approach_height
      elif approach_axis=="x":
        object_pose.pose.position.x = approach_height
      self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)  

  def pick_screw(self, robot_name, screw_size = 4, screw_number = 1):
    goal = o2as_msgs.msg.pickGoal()
    goal.robot_name = robot_name
    goal.tool_name = "screw_tool"
    goal.screw_size = screw_size
    pscrew = geometry_msgs.msg.PoseStamped()
    pscrew.header.frame_id = "tray_2_screw_m" + str(screw_size) + "_" + str(screw_number)
    pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi*11/12, 0, 0))
    goal.item_pose = pscrew
    rospy.loginfo("Sending pick action goal")
    rospy.loginfo(goal)

    self.pick_client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    self.pick_client.wait_for_result()
    rospy.loginfo("Getting result")
    self.pick_client.get_result()


  def place_plate_3_and_screw(self, place_plate_only=False, screw_first_only=False, reverse_placement_only=False):
    # Requires the screw tool to be equipped on b_bot
    self.log_to_debug_monitor("Home position", "operation")
    self.go_to_named_pose("back", "c_bot")
    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("screw_ready_back", "b_bot")

    if not screw_first_only:
      rospy.loginfo("Going to pick up plate_3 with c_bot")
      # TODO: Attach a spawned object, use its frames to plan the next motion
      # TEMPORARY WORKAROUND: Use initial+assembled position. This does not do collision avoidance!!
      self.send_gripper_command("c_bot", "open")
      ps_approach = geometry_msgs.msg.PoseStamped()
      ps_approach.header.frame_id = "initial_assy_part_03_pulley_ridge_bottom" # The top corner of the big plate
      ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, pi/2, -pi/2))
      ps_approach.pose.position.x = 0.0025
      ps_approach.pose.position.y = -0.02
      ps_approach.pose.position.z = 0.05
      ps_pickup = copy.deepcopy(ps_approach)
      ps_pickup.pose.position.z = -0.03    
      ps_high = copy.deepcopy(ps_approach)
      ps_high.pose.position.z = 0.13
      ps_place = copy.deepcopy(ps_pickup)
      ps_place.header.frame_id = "assembled_assy_part_03_pulley_ridge_bottom"
      ps_place.pose.position.z += .001
      ps_place.pose.position.x += .001 # MAGIC NUMBER!!  positive points towards c_bot
      ps_shake_off = copy.deepcopy(ps_place)
      ps_shake_off.pose.position.z -= .002
      ps_move_away = copy.deepcopy(ps_place)
      ps_move_away.pose.position.y += .08
      ps_hold = copy.deepcopy(ps_place)
      ps_hold.pose.position.y += .02

      if reverse_placement_only: # This is for finding the right initial pose of the plate
        self.log_to_debug_monitor("Find the right initial pose of the plate", "operation")
        self.go_to_named_pose("home", "c_bot")
        self.send_gripper_command("c_bot", "close")
        self.move_lin("c_bot", ps_move_away, 1.0)
        self.send_gripper_command("c_bot", "open")
        self.move_lin("c_bot", ps_place, .2)
        self.send_gripper_command("c_bot", "close")
        self.move_lin("c_bot", ps_high, 1.0)
        self.move_lin("c_bot", ps_approach, 1.0)
        ps_pickup.pose.position.z = 0.0015
        self.move_lin("c_bot", ps_pickup, 0.2)
        rospy.loginfo("Done")
        return
      
      self.go_to_named_pose("home", "c_bot")
      self.move_lin("c_bot", ps_approach, 1.0)
      self.move_lin("c_bot", ps_pickup, 1.0)
      self.send_gripper_command("c_bot", "close")
      rospy.sleep(1)

    # Pick up screw while the plate is grasped, so the cable does not interfere
    self.log_to_debug_monitor("Pickup screw", "operation")
    if not place_plate_only:
      self.go_to_named_pose("screw_pick_ready", "b_bot")
      self.pick_screw("b_bot", screw_size=4, screw_number=1)
      self.go_to_named_pose("screw_ready_back", "b_bot")

    if not screw_first_only:
      # Deliver the plate to its assembled position
      self.log_to_debug_monitor("Driver the plate", "operation")
      self.move_lin("c_bot", ps_high, 1.0)
      self.move_lin("c_bot", ps_place, .2)
      self.send_gripper_command("c_bot", 0.008)
      rospy.sleep(1.0)
      self.move_lin("c_bot", ps_shake_off, .1)
      self.send_gripper_command("c_bot", 0.015)
      # Move to the side for the screw tool to pass
      self.move_lin("c_bot", ps_move_away, .3)

      if place_plate_only:
        rospy.loginfo("Done placing the plate.")
        return True

    ###### ==========
    # Move b_bot to the first hole and screw
    self.log_to_debug_monitor("Screw the plate", "operation")
    self.go_to_named_pose("screw_plate_ready", "b_bot")
    if not screw_first_only:
      self.move_lin("c_bot", ps_hold, .1)    # Move c_bot to the plate so it does not move too much
      # self.send_gripper_command("c_bot", 0.008)

    pscrew = geometry_msgs.msg.PoseStamped()
    pscrew.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_1"
    # pscrew.pose.position.y = .00   # MAGIC NUMBER (negative goes towards c_bot)
    pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi/4, 0,0))
    pscrew_approach = copy.deepcopy(pscrew)
    pscrew_approach.pose.position.y -= .02
    pscrew_approach.pose.position.x -= .03
    self.move_lin("b_bot", pscrew_approach, speed=0.1, acceleration=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.do_screw_action("b_bot", pscrew, screw_height = 0.002, screw_size = 4)
    self.go_to_named_pose("screw_plate_ready", "b_bot")
    self.go_to_named_pose("screw_ready_back", "b_bot")

    if not screw_first_only:
      ###### ========== 
      # Recenter the plate with c_bot and then move away
      self.log_to_debug_monitor("Recenter the plate with c_bot and then move away", "operation")
      self.send_gripper_command("c_bot", "open")
      rospy.sleep(1.0)
      self.send_gripper_command("c_bot", "close")
      rospy.sleep(2.0)
      self.send_gripper_command("c_bot", "open")
      rospy.sleep(1.0)

      self.move_lin("c_bot", ps_move_away, .3)
      self.go_to_named_pose("back", "c_bot")

      ##### ========== Pick another screw with b_bot and fix the plate
      # Pick up screw from tray
      self.log_to_debug_monitor("Pick up screw from tray", "operation")
      self.go_to_named_pose("screw_pick_ready", "b_bot")
      self.pick_screw("b_bot", screw_size=4, screw_number=2)
      self.go_to_named_pose("screw_pick_ready", "b_bot")
      
      self.go_to_named_pose("screw_plate_ready", "b_bot")
      pscrew_2 = geometry_msgs.msg.PoseStamped()
      pscrew_2.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_2"
      # pscrew_2.pose.position.y = -.001   # MAGIC NUMBER  (negative goes towards c_bot)
      pscrew_2.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi/4, 0,0))
      pscrew_2_approach = copy.deepcopy(pscrew_2)
      pscrew_2_approach.pose.position.y -= .02
      pscrew_2_approach.pose.position.x -= .03
      self.move_lin("b_bot", pscrew_2_approach, speed=0.1, acceleration=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link")
      self.do_screw_action("b_bot", pscrew_2, screw_height = 0.002, screw_size = 4)
      self.go_to_named_pose("screw_plate_ready", "b_bot")

  def place_plate_2_and_screw(self):
    # Requires the tool to be equipped on b_bot
    rospy.loginfo("Going to pick up screw with b_bot")
    self.go_to_named_pose("back", "c_bot")
    self.go_to_named_pose("back", "a_bot")

    ### --- b_bot
    # Pick first screw with b_bot
    self.log_to_debug_monitor("Pick up screw with b_bot", "operation")
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    self.pick_screw("b_bot", screw_size=4, screw_number=3)
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    self.go_to_named_pose("screw_ready_back", "b_bot")

    ### --- c_bot
    # Place plate and hold
    # rospy.loginfo("Going to pick up and place motor plate with c_bot")
    self.log_to_debug_monitor("Pick up and place motor plate with c_bot", "operation")
    self.go_to_named_pose("home", "c_bot")
    self.send_gripper_command("c_bot", "open")
    ps_approach = geometry_msgs.msg.PoseStamped()
    ps_approach.header.frame_id = "initial_assy_part_02_back_hole" # The top corner of the motor plate
    ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, pi/2, pi/2))
    ps_approach.pose.position.x = 0.0025
    ps_approach.pose.position.y = 0.0
    ps_approach.pose.position.z = 0.05
    ps_pickup = copy.deepcopy(ps_approach)
    ps_pickup.pose.position.z = -0.03
    ps_high = copy.deepcopy(ps_approach)
    ps_high.pose.position.z = 0.13
    ps_place = copy.deepcopy(ps_pickup)
    ps_place.header.frame_id = "assembled_assy_part_02_back_hole"
    ps_place.pose.position.z += .001
    ps_place.pose.position.x += .001    # MAGIC NUMBER
    ps_hold = copy.deepcopy(ps_place)
    ps_hold.pose.position.y -= .035
    ps_hold.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_multiply(
                            tf.transformations.quaternion_from_euler(0, pi/2, pi/2), 
                            tf.transformations.quaternion_from_euler(0, -pi*6/180, 0) ))
    
    self.move_lin("c_bot", ps_approach, 1.0)
    self.move_lin("c_bot", ps_pickup, 1.0)
    self.send_gripper_command("c_bot", "close")
    self.move_lin("c_bot", ps_high, 1.0)

    # Go to the same pose at the assembly position
    self.move_lin("c_bot", ps_place, 1.0)
    self.send_gripper_command("c_bot", 0.008)
    rospy.sleep(1.0)
    self.send_gripper_command("c_bot", 0.08)
    self.move_lin("c_bot", ps_hold, 1.0)
    self.send_gripper_command("c_bot", 0.008)
    
    ### --- b_bot
    # Fasten motor plate with first screw
    self.log_to_debug_monitor("Fasten motor plate with first screw", "operation")
    self.go_to_named_pose("screw_plate_ready", "b_bot")
    pscrew = geometry_msgs.msg.PoseStamped()
    pscrew.header.frame_id = "assembled_assy_part_02_bottom_screw_hole_1"
    # pscrew.pose.position.y = -.002   # MAGIC NUMBER (negative goes towards c_bot)
    pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi/4, 0,0))
    pscrew_approach = copy.deepcopy(pscrew)
    pscrew_approach.pose.position.y -= .02
    pscrew_approach.pose.position.x -= .03
    self.move_lin("b_bot", pscrew_approach, speed=0.1, acceleration=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.do_screw_action("b_bot", pscrew, screw_height = 0.002, screw_size = 4)
    self.move_lin("b_bot", pscrew_approach, speed=0.1, acceleration=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.go_to_named_pose("screw_plate_ready", "b_bot")

    # Pick second screw
    self.log_to_debug_monitor("Pick up second screw", "operation")
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    self.pick_screw("b_bot", screw_size=4, screw_number=4)
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    self.go_to_named_pose("screw_ready_back", "b_bot")

    ### --- c_bot
    # Recenter the motor plate
    self.log_to_debug_monitor("Recenter motor plate", "operation")
    ps_recenter = copy.deepcopy(ps_place)
    ps_recenter.pose.position.y -= .03
    self.send_gripper_command("c_bot", "open")
    self.move_lin("c_bot", ps_recenter, 0.2)
    self.send_gripper_command("c_bot", "close")
    rospy.sleep(1.0)
    self.send_gripper_command("c_bot", "open")
    rospy.sleep(1.0)
    self.go_to_named_pose("back", "c_bot")

    ### --- b_bot
    # Fasten motor plate with second screw
    self.log_to_debug_monitor("Fasten motor plate with second screw", "operation")
    self.go_to_named_pose("screw_plate_ready", "b_bot")
    pscrew_2 = geometry_msgs.msg.PoseStamped()
    pscrew_2.header.frame_id = "assembled_assy_part_02_bottom_screw_hole_2"
    # pscrew_2.pose.position.y = -.003   # MAGIC NUMBER (negative goes towards c_bot)
    pscrew_2.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi*80/180, 0,0))
    pscrew_2_approach = copy.deepcopy(pscrew_2)
    pscrew_2_approach.pose.position.y -= .02
    pscrew_2_approach.pose.position.x -= .03
    self.move_lin("b_bot", pscrew_2_approach, speed=0.1, acceleration=0.1, end_effector_link="b_bot_screw_tool_m4_tip_link")
    self.do_screw_action("b_bot", pscrew_2, screw_height = 0.002, screw_size = 4)
    self.go_to_named_pose("screw_plate_ready", "b_bot")
    return

  # ============================================ IDLER PIN SUBTASK
  
  def pick_retainer_pin_from_tray_and_place_in_holder(self, do_centering=False):
    # rospy.loginfo("============ Pick up the retainer pin using b_bot ============")
    self.log_to_debug_monitor("Pick up the retainer pin using b_bot", "operation")
    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("home", "b_bot")

    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "tray_2_partition_2"
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    pick_pose.pose.position.z = 0.025  # MAGIC NUMBER (actually it gets ignored by pick_joshua)

    self.pick_joshua("b_bot", pick_pose, grasp_height=0.015, speed_fast=1.0, speed_slow=.1, gripper_command="", approach_height=.05)

    if do_centering:
      self.adjust_centering("b_bot")

    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "retainer_pin_holder_link"
    place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))

    self.place_joshua("b_bot", place_pose, place_height=0.01, speed_fast=.5, speed_slow=.2, gripper_command="", approach_height=.05)
    return


  def pick_retainer_pin_from_holder(self):
    # rospy.loginfo("============ Pick up the retainer pin from holder using b_bot ============")
    self.log_to_debug_monitor("Pick up the retainer pin from holder using b_bot", "operation")
    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("home", "b_bot")
    self.send_gripper_command("b_bot", "open")

    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "retainer_pin_holder_link"
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    pick_pose.pose.position.z = 0.01  # MAGIC NUMBER (increasing it makes the gripper pick the pin closer to the head)

    self.pick_joshua("b_bot", pick_pose, grasp_height=0.0, speed_fast=1.0, speed_slow=.1, gripper_command="", approach_height=.05)
    return

  def rotate_hand_facing_the_sky(self):
    # rospy.loginfo("============ making b_bot end effector face the sky ============")
    self.log_to_debug_monitor("Making b_bot end effector face the sky", "operation")
    #self.go_to_named_pose("home", "b_bot")
    self.groups["b_bot"].set_joint_value_target([1.75, -1.24, 1.28, -0.00, 0.23, -1.55])
    self.groups["b_bot"].set_max_velocity_scaling_factor(.3)
    self.groups["b_bot"].go(wait=True)
    self.groups["b_bot"].stop()

    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    
    self.go_to_pose_goal("b_bot", intermediate_retainer_pin_tip,speed=.3, move_lin = True)
    intermediate_retainer_pin_tip.pose.position.z -= 0.4
    self.go_to_pose_goal("b_bot", intermediate_retainer_pin_tip,speed=.3, move_lin = True)
    # The joint poses at the final pose are:
    # 2.053785800933838, -1.21775991121401, 2.5320937633514404, -2.8856785933123987, 1.5664243698120117, -0.48553735414613897
    return

  def pick_retainer_pin_spacer(self, robot_name = "a_bot"):
    # rospy.loginfo("============ Picking up a retainer pin spacer using a_bot ============")
    self.log_to_debug_monitor("Picking up a retainer pin spacer using a_bot", "operation")
    self.go_to_named_pose("home", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_3_pickup"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.01

    self.pick_joshua("a_bot",pose0,-0.015,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05)

    self.go_to_named_pose("home", robot_name)
    return

  def place_retainer_pin_spacer(self, robot_name = "a_bot"):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.x = -0.022
    intermediate_retainer_pin_tip.pose.position.y = self.idler_pin_handover_offset_y
    intermediate_retainer_pin_tip.pose.position.z += self.idler_pin_handover_offset_z

    self.place_joshua("a_bot",intermediate_retainer_pin_tip,-0.022,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0,approach_axis="x", lift_up_after_place = False)
    rospy.sleep(0.5)
    if self.use_real_robot:
      self.horizontal_spiral_motion("a_bot", .004)
      rospy.loginfo("doing spiral motion")

    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_facing_sky.pose.position.x = 0.
    intermediate_facing_sky.pose.position.y = 0.
    intermediate_facing_sky.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    return

  def pick_idle_pulley(self, robot_name = "a_bot"):
    # rospy.loginfo("============ Picking up the idle pulley using a_bot ============")
    self.log_to_debug_monitor("Picking up the idle pulley using a_bot", "operation")
    self.go_to_named_pose("home", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_5"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.02

    self.pick_joshua("a_bot",pose0,-0.014,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)

    self.go_to_named_pose("home", robot_name)
    return

  def place_idle_pulley(self, robot_name = "a_bot"):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.x = -0.022
    intermediate_retainer_pin_tip.pose.position.y = self.idler_pin_handover_offset_y
    intermediate_retainer_pin_tip.pose.position.z += self.idler_pin_handover_offset_z

    self.place_joshua("a_bot",intermediate_retainer_pin_tip,-0.022,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0,approach_axis="x", lift_up_after_place = False)
    rospy.sleep(0.5)
    
    if self.use_real_robot:
      self.horizontal_spiral_motion("a_bot", .004)
      rospy.loginfo("doing spiral motion")

    # Push the pulley down, in case it got stuck
    self.log_to_debug_monitor("Push the pulley down", "operation")
    push_down_pose_1 = copy.deepcopy(intermediate_retainer_pin_tip)
    push_down_pose_1.pose.position.x = -0.02
    push_down_pose_2 = copy.deepcopy(push_down_pose_1)
    push_down_pose_2.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, pi, 0))
    self.send_gripper_command("a_bot", "open")
    self.go_to_pose_goal("a_bot", push_down_pose_2,speed=.1, move_lin = True)
    self.go_to_pose_goal("a_bot", push_down_pose_1,speed=.1, move_lin = True)

    self.go_to_named_pose("home", "a_bot")
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_facing_sky.pose.position.x = 0.
    intermediate_facing_sky.pose.position.y = 0.
    intermediate_facing_sky.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    return

  def pick_retainer_pin_nut(self):
    # rospy.loginfo("============ Picking up the retainer pin nut using a_bot ============")
    self.log_to_debug_monitor("Picking up the retainer pin nut using a_bot", "operation")
    self.go_to_named_pose("home", "a_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_5_pickup"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.02

    self.pick_joshua("a_bot",pose0,-0.015,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05)

    self.go_to_named_pose("home", "a_bot")
    return

  def place_retainer_pin_nut_and_pick_with_tool(self):
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("screw_ready", "c_bot")
    nut_intermediate_a_bot = geometry_msgs.msg.PoseStamped()
    nut_intermediate_a_bot.header.frame_id = "workspace_center"
    nut_intermediate_a_bot.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    nut_intermediate_a_bot.pose.position.x = -.25
    nut_intermediate_a_bot.pose.position.y = -.20

    nut_intermediate_c_bot = copy.deepcopy(nut_intermediate_a_bot)
    nut_intermediate_c_bot.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/4))

    self.place_joshua("a_bot",nut_intermediate_a_bot,0.0,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05,approach_axis="z", lift_up_after_place = True)
    self.go_to_named_pose("back", "a_bot")
    
    self.go_to_named_pose("tool_pick_ready", "c_bot")
    self.do_change_tool_action("c_bot", equip=True, screw_size=66)
    self.go_to_named_pose("screw_ready", "c_bot")
    self.pick_nut_from_table("c_bot", object_pose=nut_intermediate_c_bot,end_effector_link="c_bot_nut_tool_m6_tip_link")
    return

  def pick_retainer_pin_washer_2(self):
    # rospy.loginfo("============ Picking up the retainer pin washer 1 using a_bot ============")
    self.log_to_debug_monitor("Picking up the retainer pin washer 1 using a_bot", "operation")

    self.go_to_named_pose("home", "a_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_8_pickup_2"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.01 # Gets ignored by pick_joshua

    self.pick_joshua("a_bot",pose0,-0.02,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05)

    self.go_to_named_pose("home", "a_bot")
    return

  def pick_retainer_pin_washer_1(self):
    # rospy.loginfo("============ Picking up the retainer pin washer 2 using a_bot ============")
    self.log_to_debug_monitor("Picking up the retainer pin washer 2 using a_bot", "operation")
    self.go_to_named_pose("home", "a_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_8_pickup_1"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.01 # Gets ignored by pick_joshua


    self.pick_joshua("a_bot",pose0,-0.02,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05)

    self.go_to_named_pose("home", "a_bot")
    return

  def place_retainer_pin_washer_1(self, robot_name = "a_bot"):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.x = -0.022
    intermediate_retainer_pin_tip.pose.position.y = self.idler_pin_handover_offset_y
    intermediate_retainer_pin_tip.pose.position.z += self.idler_pin_handover_offset_z

    self.place_joshua("a_bot",intermediate_retainer_pin_tip,-0.022,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0,approach_axis="x", lift_up_after_place = False)
    rospy.sleep(0.5)
    if self.use_real_robot:
      self.horizontal_spiral_motion("a_bot", .004)
      rospy.loginfo("doing spiral motion")

    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_facing_sky.pose.position.x = 0.
    intermediate_facing_sky.pose.position.y = 0.
    intermediate_facing_sky.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    return

  def place_retainer_pin_washer_on_table(self):
    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "workspace_center"
    place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    place_pose.pose.position.x = -.13
    place_pose.pose.position.y = -.20

    self.place_joshua("a_bot",place_pose,-0.05,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1,approach_axis="z", lift_up_after_place = False)

    self.go_to_named_pose("home", "a_bot")
    return

  def pick_retainer_pin_washer_from_table(self):
    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "workspace_center"
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    pick_pose.pose.position.x = -.13
    pick_pose.pose.position.y = -.20
    pick_pose.pose.position.z = 0

    self.pick_joshua("a_bot",pick_pose, -0.005,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.05)

    self.go_to_named_pose("home", "a_bot")
    return

  def place_retainer_pin_washer_final(self):
    assembled_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_tip.header.frame_id = "assembled_assy_part_14_screw_tip"
    assembled_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi*80/180,pi))
    assembled_retainer_pin_tip.pose.position.x = -0.007  # negative goes closer to the plate
    assembled_retainer_pin_tip.pose.position.z = -0.003  # MAGIC NUMBER? It's the vertical axis offset

    self.place_joshua("a_bot",assembled_retainer_pin_tip,0.0,
                                speed_fast = 0.3, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.08,approach_axis="z", lift_up_after_place = False)

    self.horizontal_spiral_motion("a_bot", .004)
    a_bot_retreat = copy.deepcopy(assembled_retainer_pin_tip)
    a_bot_retreat.pose.position.z += .1
    self.go_to_pose_goal("a_bot", a_bot_retreat, speed=.1, move_lin = True)

    self.go_to_named_pose("home", "a_bot")
    return

  def insert_retainer_pin_to_base(self):
    # rospy.loginfo("============ inserting retainer pin to the base plate ============")
    self.log_to_debug_monitor("Insert retainer pin to the base plate", "operation")
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "c_bot")
    
    assembled_retainer_pin_head = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_head.header.frame_id = "assembled_assy_part_14_screw_head"
    assembled_retainer_pin_head.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    assembled_retainer_pin_head.pose.position.x = 0.00
    # assembled_retainer_pin_head.pose.position.y = -0.0024  # MAGIC NUMBER
    assembled_retainer_pin_head.pose.position.z = 0.015  # Offset 

    assembled_retainer_pin_head_final=copy.deepcopy(assembled_retainer_pin_head)
    assembled_retainer_pin_head_final.pose.position.x = 0.0053 # MAGIC NUMBERS
    # assembled_retainer_pin_head_final.pose.position.y = -0.0023 # MAGIC NUMBERS
    # assembled_retainer_pin_head_final.pose.position.z = 0.0033 # MAGIC NUMBERS

    self.groups["b_bot"].set_joint_value_target([1.315, -1.24, 1.28, -0.00, 0.23, -1.55])
    self.groups["b_bot"].set_max_velocity_scaling_factor(.31)
    self.groups["b_bot"].go(wait=True)
    self.groups["b_bot"].stop()

    self.groups["b_bot"].set_joint_value_target([1.924, -0.998, 1.4937, -0.496, -2.775, -3.14])
    self.groups["b_bot"].set_max_velocity_scaling_factor(.31)
    self.groups["b_bot"].go(wait=True)
    self.groups["b_bot"].stop()

    self.go_to_pose_goal("b_bot", assembled_retainer_pin_head,speed=.05, move_lin = True)

    #self.do_linear_push("b_bot", 5, wait = True)

    self.go_to_pose_goal("b_bot", assembled_retainer_pin_head_final,speed=.05, move_lin = True)
    return
  
  def hold_idle_pulley_with_a_bot(self):
    self.log_to_debug_monitor("Hold idel pulley with a_bot", "operation")
    self.go_to_named_pose("home", "a_bot")
    hold_pose_approach = geometry_msgs.msg.PoseStamped()
    hold_pose_approach.header.frame_id = "assembled_assy_part_14_screw_head"
    hold_pose_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    hold_pose_approach.pose.position.x = -0.01   # Points away from plate  
    # hold_pose_approach.pose.position.y = -0.0024  # MAGIC NUMBER
    hold_pose_approach.pose.position.z = .01
    
    hold_pose_approach_high = copy.deepcopy(hold_pose_approach)
    hold_pose_approach_high.pose.position.z += 0.04

    hold_pose = copy.deepcopy(hold_pose_approach)
    hold_pose.pose.position.x = -0.003
    if self.use_real_robot:
      self.send_gripper_command("a_bot", "close")
      self.go_to_pose_goal("a_bot", hold_pose_approach_high, speed=.1, move_lin = True)
    self.go_to_pose_goal("a_bot", hold_pose_approach, speed=.01, move_lin = True)
    self.go_to_pose_goal("a_bot", hold_pose, speed=.01, move_lin = True)
    return
  
  def release_and_push_with_b_bot(self):
    self.log_to_debug_monitor("Release and push with b_bot", "operation")
    self.send_gripper_command("b_bot", "close")
    rospy.sleep(1.0)
    self.send_gripper_command("b_bot", "open")
    rospy.sleep(1.0)

    b_bot_going_back = geometry_msgs.msg.PoseStamped()
    b_bot_going_back.header.frame_id = "assembled_assy_part_14_screw_head"
    b_bot_going_back.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    b_bot_going_back.pose.position.x = -0.03
    # b_bot_going_back.pose.position.y = -0.0024 # MAGIC NUMBER
    b_bot_going_back.pose.position.z = 0.01


    b_bot_going_back_below = copy.deepcopy(b_bot_going_back)
    b_bot_going_back_below.pose.position.z -= 0.008

    self.go_to_pose_goal("b_bot", b_bot_going_back, speed=.1, move_lin = True)
    self.send_gripper_command("b_bot", "close")
    self.send_gripper_command("b_bot", "close")
    self.send_gripper_command("b_bot", "close")
    self.send_gripper_command("b_bot", "close")
    self.send_gripper_command("b_bot", "close")
    rospy.sleep(0.5)
    self.go_to_pose_goal("b_bot", b_bot_going_back_below, speed=.01, move_lin = True)
    self.do_linear_push("b_bot", 3, wait = True)
    return

  def release_idle_pulley_from_a_bot(self):
    self.log_to_debug_monitor("Release idle pulley from a_bot", "operation")
    assembled_retainer_pin_head_retreat = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_head_retreat.header.frame_id = "assembled_assy_part_14_screw_head"
    assembled_retainer_pin_head_retreat.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    assembled_retainer_pin_head_retreat.pose.position.x = 0
    # assembled_retainer_pin_head_retreat.pose.position.y = -0.0024 # MAGIC NUMBER
    assembled_retainer_pin_head_retreat.pose.position.z = 0.045
    self.go_to_pose_goal("a_bot", assembled_retainer_pin_head_retreat, speed=.1, move_lin = True)

    self.go_to_named_pose("home", "a_bot")
    return

  def fasten_retainer_pin_nut(self):
    # self.go_to_named_pose("back", "c_bot")
    # nut_tool_prep_pose = [0.21349821984767914, -1.6296418348895472, 1.5491323471069336, -0.07698423067201787, -0.413309399281637, -1.436751667653219]
    # self.move_joints("c_bot", nut_tool_prep_pose)
    # self.confirm_to_proceed("after joint pose")

    nut_approach = geometry_msgs.msg.PoseStamped()
    nut_approach.header.frame_id = "assembled_assy_part_14_screw_tip"
    nut_approach.pose.position.x = 0.01
    # nut_approach.pose.position.y = -0.0051578  # MAGIC NUMBER
    nut_approach.pose.position.z = 0.0
    # nut_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi, 0))  # This goes sideways, but the gripper hits the base plate
    nut_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi*180/180, pi, 0))

    at_pin_tip = copy.deepcopy(nut_approach)
    at_pin_tip.pose.position.x = 0.0
    at_pin_tip.pose.position.z = 0.0

    at_pin_end = copy.deepcopy(at_pin_tip)
    at_pin_end.pose.position.x = -0.01
    at_pin_end.pose.position.z = 0.0
    self.go_to_named_pose("home", "c_bot")
    self.go_to_pose_goal("c_bot", nut_approach, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.confirm_to_proceed("approach pose 1")
    self.go_to_pose_goal("c_bot", at_pin_tip, speed=.08, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.confirm_to_proceed("at tip")
    self.set_motor("nut_tool_m6", direction = "tighten", wait=False, speed = 500, duration = 15)
    self.do_linear_push("c_bot", 10,direction="c_bot_diagonal", wait = True)
    self.horizontal_spiral_motion("c_bot", .004, radius_increment = .002, speed = 0.02, spiral_axis="YZ")
    self.go_to_pose_goal("c_bot", at_pin_end, speed=.08, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.confirm_to_proceed("at tip full")
    self.horizontal_spiral_motion("c_bot", .004, radius_increment = .002, speed = 0.02, spiral_axis="YZ")
    rospy.sleep(3)
    self.go_to_pose_goal("c_bot", at_pin_tip, speed=.05, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.go_to_pose_goal("c_bot", nut_approach, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.go_to_named_pose("home", "c_bot")

  # ================================================= End of IDLE PULLEY subtask

  def pick_retainer_pin_and_place_in_holder(self):
    self.log_to_debug_monitor("Pick retainer pin and place in holder", "operation")
    self.confirm_to_proceed("pick_retainer_pin")
    self.pick_retainer_pin()
    self.confirm_to_proceed("adjust_centering")
    self.adjust_centering()

    self.go_to_named_pose("home", "b_bot")
    place_pose = geometry_msgs.msg.PoseStamped()
    place_pose.header.frame_id = "retainer_pin_holder_link"
    place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    self.place_joshua("b_bot", place_pose, place_height=.01, speed_fast=1.0, speed_slow=0.5, approach_height=.05)
    self.go_to_named_pose("home", "b_bot")
    return

  # =================

  # ===================START OF MOTOR ROUTINE (subtask )======================================
  def pick_motor(self):
    self.log_to_debug_monitor("Pick motor", "operation")
    self.go_to_named_pose("home", "b_bot")
    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_4"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    print pose0
    self.pick_joshua("b_bot",pose0,0.05,
                                speed_fast = 0.7, speed_slow = 0.05, gripper_command="none",
                                approach_height = 0.13)
    self.go_to_named_pose("home", "b_bot")
    return
  
  def handover_motor(self):
    self.log_to_debug_monitor("Handover motor", "operation")
    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, -pi/2, 0))
    pose1.header.frame_id = "b_bot_robotiq_85_tip_link"
    pose1.pose.position.y = 0.05
    pose1.pose.position.x = 0.115
    pose1.pose.position.z = 0.26

    pose2 = copy.deepcopy(pose1)
    pose2.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, pi/2, 0))
    pose2.pose.position.x = 0.02
    pose2.pose.position.y = 0.0
    pose2.pose.position.z = 0.0

    self.go_to_pose_goal("b_bot", pose1,speed=.3, move_lin = True)
    rospy.sleep(1)
    self.go_to_pose_goal("c_bot", pose2,speed=.3, move_lin = True)

    self.send_gripper_command(gripper="c_bot",command = "close")
    rospy.sleep(2)
    for i in range(2):
      self.send_gripper_command(gripper="b_bot",command = "open")

    self.go_to_named_pose("back", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    return

  def insert_motor(self):
    self.log_to_debug_monitor("Insert motor", "operation")
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_04_inserted_13"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, -pi/2, 0))
    pre_insertion.pose.position.x = -0.05
    self.go_to_pose_goal("c_bot", pre_insertion, speed=.3, move_lin = True)
    self.do_linear_push("c_bot", 10, direction="Y-", wait = True)
    print "inserting using linear_push in Y negative direction, please ask Felix how to call linear push for different axis using his o2as_skills server"
    # impedance control may not be necessary in this case, it is not as difficult
    # self.do_insertion(robot_name="c_bot", wait= True, horizontal=True)
    return
  
  def fasten_motor_screw(self, screw_hole_number):#for picking up and fastening a screw. need to expand this for 6 screws. I think just make 6 different functions.
    self.log_to_debug_monitor("Fasten motor screw", "operation")
    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.header.frame_id = "assembled_assy_part_02_motor_screw_hole_"+str(screw_hole_number)
    pose1.pose.orientation.w =   0.707
    pose1.pose.orientation.x =  -0.707
    pose1.pose.orientation.y = 0
    pose1.pose.orientation.z = 0    
    
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    self.pick_screw("b_bot", screw_size=3, screw_number=screw_hole_number) # I commented this because this takes a long time in simulation
    self.go_to_named_pose("screw_ready", "b_bot")
    self.go_to_pose_goal("b_bot", pose1, speed=0.3,end_effector_link="b_bot_screw_tool_m3_tip_link", move_lin=True)
    # todo: add fastening action
    return

  def pick_motor_pulley(self):
    self.log_to_debug_monitor("Pick motor pulley", "operation")
    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_6"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.07
    self.send_gripper_command(gripper="b_bot",command = 0.05)
    self.pick_joshua("b_bot", pose0, grasp_height=0.05, speed_fast=1.0, speed_slow=.1, gripper_command="", approach_height=.05)
    self.send_gripper_command(gripper="b_bot",command = "close")
    self.go_to_named_pose("home", "b_bot")
    return

  def insert_motor_pulley(self):
    self.log_to_debug_monitor("Insert motor pulley", "operation")
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_04_tip"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    pre_insertion.pose.position.x = 0.04
    self.go_to_pose_goal("b_bot", pre_insertion, speed=.3, move_lin = True)
    self.do_insertion(robot_name="b_bot", wait= True, horizontal=False)
    return

  def pick_bearing(self):
    self.log_to_debug_monitor("Pick bearing", "operation")
    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_2"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.07

    self.do_pick_action("b_bot", pose0, z_axis_rotation = 0.0, use_complex_planning = False)
    return

  def pick_end_cap(self):
    self.log_to_debug_monitor("Pick end cap", "operation")
    self.go_to_named_pose("home", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("home", "b_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    self.go_to_named_pose("back", "c_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)
    self.send_gripper_command("b_bot", "open")

    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_4"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.07
    self.send_gripper_command(gripper="b_bot",command = 0.04)
    self.pick_joshua("b_bot", pose0, grasp_height=0.02, speed_fast=1.0, speed_slow=.1, gripper_command="", approach_height=.05)
    self.send_gripper_command(gripper="b_bot",command = "close")
    self.go_to_named_pose("home", "b_bot")
      
    handover_b = geometry_msgs.msg.PoseStamped()
    handover_b.header.frame_id = "workspace_center"
    handover_b.pose.position.x = 0.2
    handover_b.pose.position.y = .0
    handover_b.pose.position.z = 0.7
    handover_b.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, -pi/2))
      

    handover_a_approach = copy.deepcopy(handover_b)
    handover_a_approach.pose.position.y -= 0.05
    handover_a_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, pi/2))
    
    self.go_to_pose_goal("b_bot", handover_b, speed=0.2)
    self.go_to_pose_goal("a_bot", handover_a_approach, speed=0.12)

    # Applied to the a_bot in workspace_center coordinates
    magic_x_offset = .001  
    magic_z_offset = .001

    handover_a  = copy.deepcopy(handover_a_approach)
    handover_a.pose.position.y += 0.05
    handover_a.pose.position.y += 0.023  # This is how much the gripper is pushed in
    handover_a.pose.position.x += magic_x_offset # MAGIC
    handover_a.pose.position.z += magic_z_offset # MAGIC

    self.send_gripper_command(gripper="a_bot", command="close")
    self.go_to_pose_goal("a_bot", handover_a, speed=0.03, move_lin= True)
    self.confirm_to_proceed("Is the gripper centered and in the end cap?")
    self.horizontal_spiral_motion("a_bot", .003, radius_increment = .001)
    self.send_gripper_command(gripper="a_bot", command="open")
      
    handover_b_retreat = copy.deepcopy(handover_a)
    handover_b_retreat.pose.position.y -= 0.017
    handover_b_retreat.pose.position.x -= magic_x_offset # MAGIC
    handover_b_retreat.pose.position.z -= magic_z_offset #MAGIC
    handover_b_retreat.pose.position.y += 0.03
    handover_b_retreat.pose.orientation = handover_b.pose.orientation
      
    self.send_gripper_command(gripper="b_bot", command="open")
    rospy.sleep(1.0)
    self.go_to_pose_goal("b_bot", handover_b_retreat, speed=0.02, move_lin= True)
    self.go_to_named_pose("back", "b_bot")
    self.go_to_named_pose("home", "a_bot")

  def place_end_cap(self):
    rospy.logerr("function place_end_cap is not implemented.aborting")
    return
    self.place_joshua("a_bot", self.place_poses[i-1],self.item_place_heights[i-1],
                              speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                              approach_height = 0.05, lift_up_after_place = False)

    #  self.send_gripper_command(gripper="precision_gripper_inner", command="open")
    self.horizontal_spiral_motion("a_bot", .001, radius_increment = .001)
    p = copy.deepcopy(self.place_poses[i-1])
    p.pose.position.z += 0.02
    self.go_to_pose_goal("a_bot", p, speed=0.02, move_lin= True)
    self.go_to_named_pose("home", "a_bot", speed=2.0, acceleration=2.0, force_ur_script=self.use_real_robot)

  def pick_clamping_shaft(self):
    self.log_to_debug_monitor("Pick motor pulley", "operation")
    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_1_pickup"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.07
    self.send_gripper_command(gripper="b_bot",command = 0.05)
    self.pick_joshua("b_bot", pose0, grasp_height=0.05, speed_fast=1.0, speed_slow=.1, gripper_command="", approach_height=.05)
    self.send_gripper_command(gripper="b_bot",command = "close")
    self.go_to_named_pose("home", "b_bot")
    return

  def insert_bearing(self):
    self.log_to_debug_monitor("Insert bearing", "operation")
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_11_front_hole"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pre_insertion.pose.position.x = -0.04
    self.go_to_pose_goal("b_bot", pre_insertion, speed=.3, move_lin = True)

  def subtask_f(self):
    # rospy.loginfo("======== SUBTASK F (motor plate) ========")
    self.log_to_debug_monitor("SUBTASK F (motor plate)", "subtask")
    self.log_to_debug_monitor("=== Subtask F start ===", "operation")
    self.place_plate_2_and_screw()
    self.log_to_debug_monitor("=== Subtask F end ===", "operation")

  def subtask_g(self):
    # ===== SUBTASK G (Placing and fastening the output (large) plate, for idle pulley set and clamping pulley set) =========
    # rospy.loginfo("======== SUBTASK G (large plate) ========")
    self.log_to_debug_monitor("SUBTASK G (large plate)", "subtask")
    self.log_to_debug_monitor("=== Subtask G start ===", "operation")
    self.place_plate_3_and_screw()
    rospy.loginfo("todo: add screw picking and fastening sequence for the second screw")
    self.log_to_debug_monitor("=== Subtask G end ===", "operation")

  def subtask_a(self):
    # ============= SUBTASK A (picking and inserting and fastening the motor shaft) =======================
    # rospy.loginfo("======== SUBTASK A (motor) ========")
    self.log_to_debug_monitor("SUBTASK A (motor)", "subtask")
    self.log_to_debug_monitor("=== Subtask F start ===", "operation")
    self.pick_motor()
    self.adjust_centering()
    self.handover_motor()
    rospy.loginfo("todo: pick up m3 tool using b bot, replace this print with the equip function")
    self.insert_motor() # Joshua thinks this may be possible to do without impedance control, otherwise use insertion script in Y negative direction
    self.go_to_named_pose("screw_ready", "b_bot")
    for i in range(1,7):
      self.fasten_motor_screw(screw_hole_number=i) # please ask Felix about the pick_screw function, I am not sure how he defined it
    self.log_to_debug_monitor("=== Subtask A end ===", "operation")

  def subtask_b(self):
    # ================================= SUBTASK B (motor pulley) ===========================================
    # rospy.loginfo("======== SUBTASK B (motor pulley) ========")
    self.log_to_debug_monitor("SUBTASK B (motor pulley)", "subtask")
    self.log_to_debug_monitor("=== Subtask B start ===", "operation")
    self.pick_motor_pulley()
    self.insert_motor_pulley()
    rospy.loginfo("todo: fasten motor pulley") # With the set screw
    self.log_to_debug_monitor("=== Subtask B end ===", "operation")

  
  def subtask_e(self):
    # ======================== SUBTASK E (The idler pin) ============================================
    # rospy.loginfo("======== SUBTASK E ========")
    self.log_to_debug_monitor("SUBTASK E", "subtask")
    self.log_to_debug_monitor("=== Subtask E start ===", "operation")
    # ====== (This is the first thing to do in the task)
    # self.confirm_to_proceed("pick_retainer_pin_from_tray_and_place_in_holder")
    # self.pick_retainer_pin_from_tray_and_place_in_holder()
    # ====== 
    self.confirm_to_proceed("pick_retainer_pin_from_holder")
    self.pick_retainer_pin_from_holder()
    self.go_to_named_pose("home", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)
    self.confirm_to_proceed("adjust_centering")
    self.adjust_centering()
    self.go_to_named_pose("home", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)
    self.confirm_to_proceed("rotate_hand_facing_the_sky")
    self.rotate_hand_facing_the_sky()
    self.confirm_to_proceed("pick_idle_pulley")
    self.pick_idle_pulley()
    self.confirm_to_proceed("place_idle_pulley")
    self.place_idle_pulley()
    self.confirm_to_proceed("pick_retainer_pin_spacer")
    self.pick_retainer_pin_spacer()
    self.confirm_to_proceed("place_retainer_pin_spacer")
    self.place_retainer_pin_spacer()
    self.confirm_to_proceed("pick_retainer_pin_washer_1")
    self.pick_retainer_pin_washer_1()
    self.confirm_to_proceed("place_retainer_pin_washer_1")
    self.place_retainer_pin_washer_1()
    self.confirm_to_proceed("pick_retainer_pin_nut")
    self.pick_retainer_pin_nut()
    self.confirm_to_proceed("place_retainer_pin_nut_and_pick_with_tool")
    self.place_retainer_pin_nut_and_pick_with_tool()
    self.confirm_to_proceed("pick_retainer_pin_washer_2")
    self.pick_retainer_pin_washer_2()
    self.confirm_to_proceed("place_retainer_pin_washer_on_table")
    self.place_retainer_pin_washer_on_table()
    self.confirm_to_proceed("insert_retainer_pin_to_base")
    self.insert_retainer_pin_to_base()
    self.confirm_to_proceed("hold_idle_pulley_with_a_bot")
    self.hold_idle_pulley_with_a_bot()
    self.confirm_to_proceed("release_and_push_with_b_bot")
    self.release_and_push_with_b_bot()
    self.confirm_to_proceed("release_idle_pulley_from_a_bot")
    self.release_idle_pulley_from_a_bot()
    self.confirm_to_proceed("pick_retainer_pin_washer_from_table")
    self.pick_retainer_pin_washer_from_table()
    self.confirm_to_proceed("place_retainer_pin_washer_final")
    self.place_retainer_pin_washer_final()
    self.confirm_to_proceed("fasten_retainer_pin_nut")
    self.fasten_retainer_pin_nut()
    self.do_change_tool_action("c_bot", equip=False, screw_size=66)
    self.log_to_debug_monitor("=== Subtask E end ===", "operation")

  def subtask_c(self):
    # ==== SUBTASK C (clamping pulley set, everything but inserting and fastening clamping pulley) =================
    # rospy.loginfo("======== SUBTASK C (bearing + shaft) ========")
    self.log_to_debug_monitor("SUBTASK C (bearing + shaft)", "subtask")
    self.log_to_debug_monitor("=== Subtask C start ===", "operation")
    # self.pick_bearing()
    # self.insert_bearing()
    # rospy.loginfo("todo: pick up shaft, pick up cap, insert the cap, fasten the cap, insert using impedance Y negative direction (using b_bot)")
    # rospy.loginfo("todo: make the gripper not fully open while approaching for picking, may be important for motor pulley and clamping_shaft_spacer")
    self.pick_end_cap()
    self.pick_clamping_shaft()
    self.place_end_cap()
    # self.pick_shaft_spacer()
    # self.insert_shaft_spacer()
    self.log_to_debug_monitor("=== Subtask C end ===", "operation")


  def real_assembly_task(self):
    # self.start_task_timer()
    # self.log_to_debug_monitor(text="Assembly", category="task")

    # # To prepare subtask E
    # self.pick_retainer_pin_from_tray_and_place_in_holder(do_centering=False)
    # self.go_to_named_pose("home", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)

    # # To equip screw tool for subtasks G, F
    # self.go_to_named_pose("back", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)
    # self.do_change_tool_action("b_bot", equip=True, screw_size=4)
    # self.go_to_named_pose("screw_pick_ready", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)

    # self.subtask_g()  # Large plate
    # self.subtask_f() # motor plate

    # assy.go_to_named_pose("back", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
    # assy.do_change_tool_action("b_bot", equip=False, screw_size=4)

    # self.go_to_named_pose("home", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)
    # self.go_to_named_pose("home", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=self.use_real_robot)

    # self.subtask_e() #idle pulley

    # self.subtask_a() # motor

    # #with insertion script!!!only for second trial
    # self.subtask_b() # 

    self.subtask_c() # bearing, clamping pulley set

    return

if __name__ == '__main__':
  try:
    rospy.loginfo("Please refer to this page for details of each subtask. https://docs.google.com/spreadsheets/d/1Os2CfH80A7vzj6temt5L8BYpLvHKBzWT0dVuTvpx5Mk/edit#gid=1216221803")
    assy = AssemblyClass()
    assy.set_up_item_parameters()
    i = 1
    while i:
      rospy.loginfo("Enter 11 (12) to equip (unequip) m4 tool (b_bot).")
      rospy.loginfo("Enter 13 (14) to equip (unequip) m3 tool (b_bot).")
      rospy.loginfo("Enter 15 (16) to equip (unequip) m6 nut tool (c_bot).")
      rospy.loginfo("Enter 2 to move the robots home to starting positions.")
      rospy.loginfo("Enter 31-36 to pick screw m3 from tray with b_bot (number 1-6).")
      rospy.loginfo("Enter 41-49 to pick screw m4 from tray with b_bot (number 1-9).")
      rospy.loginfo("Enter 91-94 for subtasks (Large plate, motor plate, idler pin, motor).")
      rospy.loginfo("Enter 95-98 for subtasks (motor pulley, bearing+shaft, clamp pulley, belt).")
      rospy.loginfo("Enter 911 to place plate 3 (but don't screw)")
      rospy.loginfo("Enter 912 to place plate 3 from the base plate and put it on the table")
      rospy.loginfo("Enter 913 to screw in plate 3 (but don't place it)")
      rospy.loginfo("Enter 92 to place plate 2 and screw")
      rospy.loginfo("Enter 93 to do idle pulley set")
      rospy.loginfo("Enter START to start the task.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '11':
        assy.go_to_named_pose("back", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=True, screw_size=4)
      if i == '12':
        assy.go_to_named_pose("back", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=False, screw_size=4)
      if i == '13':
        assy.go_to_named_pose("back", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=True, screw_size=3)
      if i == '14':
        assy.go_to_named_pose("back", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("b_bot", equip=False, screw_size=3)
      if i == '15':
        assy.go_to_named_pose("tool_pick_ready", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("c_bot", equip=True, screw_size=66)
      if i == '16':
        assy.go_to_named_pose("tool_pick_ready", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
        assy.do_change_tool_action("c_bot", equip=False, screw_size=66)
      if i == '2':
        assy.go_to_named_pose("back", "a_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
        assy.go_to_named_pose("home", "c_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
        assy.go_to_named_pose("home", "b_bot", speed=3.0, acceleration=3.0, force_ur_script=assy.use_real_robot)
      elif i in ['31', '32', '33', '34', '35', '36']:
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
        assy.pick_screw("b_bot", screw_size=3, screw_number=int(i)-30)
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
      elif i in ['41', '42', '43', '44', '45', '46', '47', '48', '49']:
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
        assy.pick_screw("b_bot", screw_size=4, screw_number=int(i)-40)
        assy.go_to_named_pose("screw_pick_ready", "b_bot")
      elif i == '91':
        assy.subtask_g()  # Large plate
      elif i == '911':
        assy.place_plate_3_and_screw(place_plate_only=True)
      elif i == '912':
        assy.place_plate_3_and_screw(reverse_placement_only=True)
      elif i == '913':
        assy.place_plate_3_and_screw(screw_first_only=True)
      elif i == '92':
        assy.subtask_f()  # Motor plate
      elif i == '93':
        assy.subtask_e()  # Idler pin
      elif i == '94':
        assy.subtask_a() #
      elif i == 'START' or i == 'start' or i == "9999":
        for i in [1,2]:
          rospy.loginfo("Starting set number " + str(i))
          assy.real_assembly_task()
          rospy.loginfo("SET NUMBER " + str(i) + " COMPLETED. PUT THE ROBOT IN PAUSE MODE AND REPLACE THE PARTS")
          raw_input()
          if rospy.is_shutdown():
            rospy.loginfo("ABORTING")
            break
      elif i == 'x':
        break
  except rospy.ROSInterruptException:
    pass
