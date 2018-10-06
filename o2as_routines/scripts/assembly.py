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

  def set_up_item_parameters(self):
    # TODO: Publish the items to the scene, or do something equivalent. 
    self.item_names = []
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 
    

  ################ ----- Routines  
  ################ 
  ################ 
  def pick(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.03, end_effector_link=""):
    #initial gripper_setup
    rospy.loginfo("Going above object to pick")
    object_pose.pose.position.z = approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast,end_effector_link=end_effector_link, move_lin=True)

    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_innerX":
      self.precision_gripper_inner_close()
    else: 
      rospy.logerr("No gripper command was set")

    rospy.loginfo("Moving down to object")
    object_pose.pose.position.z = grasp_height
    rospy.loginfo(grasp_height)
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True,end_effector_link=end_effector_link, move_lin=True)

    # W = raw_input("waiting for the gripper")
    #gripper close
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_close(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="easy_pick_only_innerX":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
    rospy.sleep(2)
    rospy.loginfo("Going back up")
    object_pose.pose.position.z = (approach_height)
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast,end_effector_link=end_effector_link, move_lin=True)

######

  def place(self,robotname, object_pose, place_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, approach_axis="z" ,lift_up_after_place = True):
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
    elif gripper_command=="easy_pick_only_innerX":
      self.precision_gripper_inner_close()
    else: 
      rospy.logerr("No gripper command was set")
    
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
    pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi/2, 0,0))
    goal.item_pose = pscrew
    rospy.loginfo("Sending pick action goal")
    rospy.loginfo(goal)

    self.pick_client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    self.pick_client.wait_for_result()
    rospy.loginfo("Getting result")
    self.pick_client.get_result()

    self.pick_client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    self.pick_client.wait_for_result()
    rospy.loginfo("Getting result")
    self.pick_client.get_result()
  def assembly_task(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")
    # TODO

  def handover_demo(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "workspace_center"
    ps.pose.orientation.w = 1.0
    ps.pose.position.x = -.32
    ps.pose.position.y = -.315
    ps.pose.position.z = .025
    self.do_pick_action("c_bot", ps)
    self.go_to_named_pose("home", "c_bot")
    self.do_regrasp(giver_robot_name="c_bot", receiver_robot_name="b_bot", grasp_distance = -.01)

    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "c_bot")

  def insertion_demo(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")
    
    # Pick the bearing
    pick_bearing = False
    pick_bearing = True
    if pick_bearing:
      ps = geometry_msgs.msg.PoseStamped()
      ps.header.frame_id = "workspace_center"
      ps.pose.orientation.w = 1.0
      ps.pose.position.x = -.32
      ps.pose.position.y = -.315
      ps.pose.position.z = .025
      self.do_pick_action("c_bot", ps)
      self.go_to_named_pose("home", "c_bot")

    # Pick the rod
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "workspace_center"
    ps.pose.orientation.w = 1.0
    ps.pose.position.x = -.197
    ps.pose.position.y = .333
    ps.pose.position.z = .06
    self.do_pick_action("b_bot", ps)
    self.go_to_named_pose("home", "b_bot")

    # Move to the insertion pose and do it
    self.do_insert_action(active_robot_name = "b_bot", passive_robot_name = "c_bot",
                          starting_offset = .05, max_approach_distance = .1,
                          max_radius = 0.01, radius_increment = .0008)
  
  def belt_demo(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")
    
    # Go with b_bot to the belt, pick, pull
    rospy.loginfo("Going to belt with b_bot")
    psb = geometry_msgs.msg.PoseStamped()
    psb.header.frame_id = "assembled_assy_part_05_center" # The middle of the idler pulley
    psb.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(pi/2, 0.0, 0.0))
    # psb.pose.orientation.w = 1.0
    psb.pose.position.x = -0.05
    psb.pose.position.y = 0.05
    psb.pose.position.z = 0.0
    self.go_to_pose_goal("b_bot", psb, 1.0)

    # Move in and grasp
    psb.pose.position.x = 0.005
    self.go_to_pose_goal("b_bot", psb, 0.05)
    self.send_gripper_command("b_bot", "close")

    # Move belt out of the way
    psb.pose.position.z += 0.01
    self.go_to_pose_goal("b_bot", psb, 0.05)

    psb.pose.position.x = -0.02
    self.go_to_pose_goal("b_bot", psb, 0.05)

    # psb.pose.position.y = 0.0
    # self.go_to_pose_goal("b_bot", psb, 0.05)
    psb.pose.position.z = 0.0
    self.go_to_pose_goal("b_bot", psb, 0.05)


    # Grasp the pulley, move it up
    psc = geometry_msgs.msg.PoseStamped()
    psc.header.frame_id = "assembled_assy_part_14_screw_tip" # The tip of the retainer pin
    psc.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0.0, pi))
    

    # Go to approach position, prepare gripper
    psc.pose.position.x = -0.05
    self.go_to_pose_goal("c_bot", psc, 0.05)
    self.send_gripper_command("c_bot", command=.02)
    
    # Move in, grasp, move up
    psc.pose.position.x = 0.005
    self.go_to_pose_goal("c_bot", psc, 0.05)
    self.send_gripper_command("c_bot", "close")
    psc.pose.position.z += 0.02
    self.go_to_pose_goal("c_bot", psc, 0.02)

    # Put the belt under the pulley, move gripper to the left, put the pulley down a bit
    psb.pose.position.x = 0.005
    self.go_to_pose_goal("b_bot", psb, 0.05)
    # Move the pulley down
    psc.pose.position.z = 0.005
    self.go_to_pose_goal("c_bot", psc, 0.02)

    # Release the belt
    self.send_gripper_command("b_bot", "open")
    psb.pose.position.x = -0.05
    self.go_to_pose_goal("b_bot", psb, 1.0)

    #Move b_bot above the pulley, open slightly, move down via linear_search until a certain resistance force is encountered
    psb.pose.position.x = 0.0
    psb.pose.position.y = 0.0
    psb.pose.position.z = 0.04
    psb.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0.0, pi/2, 0.0)) # Facing downward on the pulley
    self.go_to_pose_goal("b_bot", psb, 1.0)
    self.send_gripper_command("b_bot", command=0.1)
    self.do_linear_push("b_bot")

    # Calculate position of pulley (it is touching both b_bot fingers)
    pulley_pose = geometry_msgs.msg.PoseStamped()
    pulley_pose.header.frame_id = "b_bot_robotiq_85_tip_link"
    pulley_pose.pose.position.x = .01   # Minus a little bit because the gripper is open
    pulley_pose.pose.position.z = .01
    pulley_pose.pose.orientation.w = 1.0
    pulley_pose = self.listener.transformPose("assembled_assy_part_14_screw_tip", pulley_pose)
    # The z-component now holds the correct height of the pulley center.
    pulley_pose.pose.position.x = -0.005
    pulley_pose.pose.position.y = 0.0
    pulley_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0.0, pi/2, 0.0))

    # Move c_bot to nut, grasp, turn
    pulley_pose.pose.position.x = -0.03
    self.go_to_pose_goal("c_bot", psc, 0.05)
    self.send_gripper_command("c_bot", command=.02)
    psc.pose.position.x = 0.005
    self.go_to_pose_goal("c_bot", psc, 0.05)
    self.send_gripper_command("c_bot", "close")

    # TODO: Turn gripper, or use fastening tool to tighten the nut (the latter would be better)

    # self.go_to_named_pose("home", "b_bot")
    # self.go_to_named_pose("home", "c_bot")
    
  def place_plate_3_and_screw(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "a_bot")

    # Pick up screw from tray
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    self.pick_screw("b_bot", screw_size=4, screw_number=1)
    rospy.sleep(1)
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    ###### ===========
    
    rospy.loginfo("Going to pick up plate_3 with c_bot")
    # TODO: Attach a spawned object, use its frames to plan the next motion
    # TEMPORARY WORKAROUND: Use initial+assembled position. This does not do collision avoidance!!
    self.send_gripper_command("c_bot", "open")
    ps_approach = geometry_msgs.msg.PoseStamped()
    ps_approach.header.frame_id = "initial_assy_part_03_pulley_ridge_bottom" # The top corner of the big plate
    ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    ps_approach.pose.position.x = 0.0025
    ps_approach.pose.position.y = 0.0
    ps_approach.pose.position.z = 0.05
    ps_approach = copy.deepcopy(ps_approach)
    ps_pickup = copy.deepcopy(ps_approach)
    ps_pickup.pose.position.z = -0.03    
    ps_high = copy.deepcopy(ps_approach)
    ps_high.pose.position.z = 0.13
    ps_place = copy.deepcopy(ps_pickup)
    ps_place.header.frame_id = "assembled_assy_part_03_pulley_ridge_bottom"
    ps_place.pose.position.z += .001
    ps_place.pose.position.x += .001 # MAGIC NUMBER!!
    ps_move_away = copy.deepcopy(ps_place)
    ps_move_away.pose.position.y += .06
    
    self.move_lin("c_bot", ps_approach, 1.0)

    self.move_lin("c_bot", ps_pickup, 1.0)
    self.send_gripper_command("c_bot", "close")
    rospy.sleep(1)
    # raw_input() # Uncomment this to draw the contour as it is grasped

    self.move_lin("c_bot", ps_high, 1.0)

    # Deliver the item to its assembled position
    self.move_lin("c_bot", ps_place, .2)
    self.send_gripper_command("c_bot", 0.008)
    # self.send_gripper_command("c_bot", 0.01)

    # Move out of the way
    self.move_lin("c_bot", ps_move_away, .3)
    # self.go_to_named_pose("back", "c_bot")

    ###### ==========
    # Move b_bot to the hole and screw
    self.go_to_named_pose("screw_plate_ready", "b_bot")

    self.move_lin("c_bot", ps_place, .1)
    # self.send_gripper_command("c_bot", 0.008)

    pscrew = geometry_msgs.msg.PoseStamped()
    pscrew.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_1"
    pscrew.pose.position.y = .004
    pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi/4, 0,0))
    self.do_screw_action("b_bot", pscrew, screw_height = 0.002, screw_size = 4)
    self.go_to_named_pose("screw_plate_ready", "b_bot")


    ###### ========== 
    # Center the plate with c_bot again and then move away
    self.send_gripper_command("c_bot", "open")
    rospy.sleep(1.0)
    self.send_gripper_command("c_bot", "close")
    rospy.sleep(2.0)
    self.send_gripper_command("c_bot", "open")
    rospy.sleep(1.0)

    self.move_lin("c_bot", ps_move_away, .3)
    self.go_to_named_pose("home", "c_bot")

    ###### ========== Pick another screw with b_bot and fix the plate
    # Pick up screw from tray
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    self.pick_screw("b_bot", screw_size=4, screw_number=2)
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    
    self.go_to_named_pose("screw_plate_ready", "b_bot")
    pscrew.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_2"
    self.do_screw_action("b_bot", pscrew, screw_height = 0.002, screw_size = 4)
    self.go_to_named_pose("screw_plate_ready", "b_bot")

    self.go_to_named_pose("screw_ready", "b_bot")

  def place_plate_2(self):
    # Requires the tool to be equipped on b_bot
    
    rospy.loginfo("Going to pick up and place plate_2 with c_bot")
    
    self.go_to_named_pose("home", "c_bot")
    self.send_gripper_command("c_bot", "open")
    ps_approach = geometry_msgs.msg.PoseStamped()
    ps_approach.header.frame_id = "initial_assy_part_02_back_hole" # The top corner of the big plate
    ps_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, pi/2, pi/2))
    ps_approach.pose.position.x = 0.0025
    ps_approach.pose.position.y = 0.0
    ps_approach.pose.position.z = 0.05
    ps_approach = copy.deepcopy(ps_approach)
    ps_pickup = copy.deepcopy(ps_approach)
    ps_pickup.pose.position.z = -0.03
    ps_high = copy.deepcopy(ps_approach)
    ps_high.pose.position.z = 0.13

    self.move_lin("c_bot", ps_approach, 1.0)

    self.move_lin("c_bot", ps_pickup, 1.0)
    self.send_gripper_command("c_bot", "close")
    # raw_input() # Uncomment this to draw the contour as it is grasped

    self.move_lin("c_bot", ps_high, 1.0)

    # Go to the same pose at the assembly position
    ps_pickup.header.frame_id = "assembled_assy_part_02_back_hole"
    ps_pickup.pose.position.z += .001
    self.move_lin("c_bot", ps_pickup, .02)
    self.send_gripper_command("c_bot", 0.008)

    rospy.sleep(.5)
    self.send_gripper_command("c_bot", 0.08)
    ps_move_away = copy.deepcopy(ps_pickup)
    ps_move_away.pose.position.x -= .01
    ps_move_away.pose.position.y -= .06
    self.move_lin("c_bot", ps_move_away, 1.0)
    ps_move_away.pose.position.x -= .1
    self.move_lin("c_bot", ps_move_away, 1.0)
    self.go_to_named_pose("home", "c_bot")


    # # ==========
    # # Move b_bot to the hole and screw
    # pscrew = geometry_msgs.msg.PoseStamped()
    # pscrew.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_1" # The top corner of the big plate
    # pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0,0))
    # # pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(pi/2, 0,0))
    # self.do_screw_action("b_bot", pscrew, screw_height = 0.02, screw_size = 4)

  def pick_retainer_pin(self, robot_name = "b_bot"):
    rospy.loginfo("============ Picking up a retainer pin using b_bot ============")
    # if robot_name=="b_bot":
    #   self.go_to_named_pose("back", "c_bot")
    # elif robot_name=="c_bot":
    #   self.go_to_named_pose("back", "b_bot")

    self.go_to_named_pose("home", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_2"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.012

    # if robot_name=="b_bot":
    #   self.go_to_pose_goal(robot_name, pose0,speed=.05, move_lin = True)
    #   # pose0.pose.position.x = -.01
    #   rospy.sleep(1.0)
    # pose0.pose.position.z =0
    # self.send_gripper_command(gripper="b_bot",command = 0.04)
    self.do_pick_action(robot_name, pose0, z_axis_rotation = 0.0, use_complex_planning = False)
    return

  def rotate_hand_facing_the_sky(self):
    rospy.loginfo("============ making b_bot end effector face the sky ============")
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
    return

  def adjust_centering(self, robot_name = "b_bot"):
    rospy.loginfo("============ Adjusting the position of the pin/shaft ============")
    self.go_to_named_pose("home", robot_name)
    self.send_gripper_command(gripper="c_bot",command = "open")

    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pose1.header.frame_id = "b_bot_robotiq_85_tip_link"
    pose1.pose.position.y = -0.15
    pose1.pose.position.z = 0.15
    self.go_to_pose_goal("b_bot", pose1,speed=.3, move_lin = True)

    rospy.sleep(1)

    pose2 = geometry_msgs.msg.PoseStamped()
    pose2.header.frame_id = "b_bot_robotiq_85_tip_link"
    pose2.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2,0,pi/2))
    pose2.pose.position.z = 0.004
    pose2.pose.position.y = 0.025
    pose2.pose.position.x = 0.015
    self.go_to_pose_goal("c_bot", pose2,speed=.3, move_lin = True)

    # self.send_gripper_command(gripper="b_bot",command = "close", velocity = .015, force = 1.0)
    self.send_gripper_command(gripper="c_bot",command = "close")
    rospy.sleep(1)
    self.send_gripper_command(gripper="b_bot",command = "open")
    # self.send_gripper_command(gripper="b_bot",command = .03)
    rospy.sleep(2)
    self.send_gripper_command(gripper="b_bot",command = "close", velocity = .05, force = 1.0)
    # self.send_gripper_command(gripper="b_bot",command = "close", force = 1.0)
    rospy.sleep(3)
    self.send_gripper_command(gripper="c_bot",command = "open")
    rospy.sleep(1)

    pose3 = geometry_msgs.msg.PoseStamped()
    pose3.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
    pose3.header.frame_id = "b_bot_robotiq_85_tip_link"
    pose3.pose.position.y = 0
    pose3.pose.position.z = 0
    self.go_to_pose_goal("b_bot", pose3,speed=.31, move_lin = True)

    self.send_gripper_command(gripper="c_bot",command = "close")
    rospy.sleep(1)
    self.send_gripper_command(gripper="b_bot",command = "open")
    # self.send_gripper_command(gripper="b_bot",command = .03)
    rospy.sleep(2)
    self.send_gripper_command(gripper="b_bot",command = "close")
    rospy.sleep(2)
    self.send_gripper_command(gripper="c_bot",command = "open")
    rospy.sleep(1)

    self.go_to_named_pose("home", "c_bot")
    return

  def pick_retainer_pin_spacer(self, robot_name = "a_bot"):
    rospy.loginfo("============ Picking up a retainer pin spacer using a_bot ============")

    self.go_to_named_pose("home", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_3"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.02

    self.pick("a_bot",pose0,-0.015,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)

    self.go_to_named_pose("home", robot_name)
    return

  def place_retainer_pin_spacer(self, robot_name = "a_bot", using_real_robot=True):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.x = -0.022
    intermediate_retainer_pin_tip.pose.position.y = 0.001
    intermediate_retainer_pin_tip.pose.position.z += 0.008

    self.place("a_bot",intermediate_retainer_pin_tip,-0.022,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0,approach_axis="x", lift_up_after_place = False)
    rospy.sleep(0.5)
    if using_real_robot is True:
      self.horizontal_spiral_motion("a_bot", .004)
      rospy.loginfo("doing spiral motion")

    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_facing_sky.pose.position.x = 0.
    intermediate_facing_sky.pose.position.y = 0.
    intermediate_facing_sky.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    return

  def pick_idle_pulley(self, robot_name = "a_bot"):
    rospy.loginfo("============ Picking up the idle pulley using a_bot ============")

    self.go_to_named_pose("home", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_5"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.02

    self.pick("a_bot",pose0,-0.014,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)

    self.go_to_named_pose("home", robot_name)
    return

  def place_idle_pulley(self, robot_name = "a_bot", using_real_robot=True):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.x = -0.022
    intermediate_retainer_pin_tip.pose.position.y = 0.001
    intermediate_retainer_pin_tip.pose.position.z += 0.008

    self.place("a_bot",intermediate_retainer_pin_tip,-0.022,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0,approach_axis="x", lift_up_after_place = False)
    rospy.sleep(0.5)
    
    if using_real_robot is True:
      self.horizontal_spiral_motion("a_bot", .004)
      rospy.loginfo("doing spiral motion")

    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_facing_sky.pose.position.x = 0.
    intermediate_facing_sky.pose.position.y = 0.
    intermediate_facing_sky.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    return

  def pick_retainer_pin_nut(self):
    rospy.loginfo("============ Picking up the retainer pin nut using a_bot ============")

    self.go_to_named_pose("home", "a_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_5"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.02

    self.pick("a_bot",pose0,-0.015,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)

    self.go_to_named_pose("home", "a_bot")
    return

  def pick_nut_with_spiral_search(self,max_radius=0.005,rotation_axis="z",object_pose="",grasp_height=0.0,speed_fast=0.31,speed_slow=0.05,approach_height=0.1,end_effector_link=""):
    real_radius=0
    x_original=object_pose.pose.position.x
    y_original=object_pose.pose.position.y
    z_original=object_pose.pose.position.z

    while(real_radius<max_radius):
      self.pick(robotname="c_bot",object_pose=object_pose,grasp_height=grasp_height,
                                  speed_fast = speed_fast, speed_slow = speed_slow, gripper_command="not_going_to_do_anything",
                                  approach_height = approach_height,end_effector_link=end_effector_link)
      real_radius=10000
    # todo [IMPORTANT]: implement a spiral search like the ones for picking up screws

    # ROS_INFO("Adjusting the position of the pick attempt slightly and retrying");
    # theta=theta+theta_incr;
    # y=cos(theta)*r;
    # z=sin(theta)*r;
    # adjusted_pose = search_start_pose;
    # adjusted_pose.pose.position.y += y;
    # adjusted_pose.pose.position.z += z;
    # r = r + radius_inc_set;
    # RealRadius = sqrt(pow(y,2)+pow(z,2));

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

    self.place("a_bot",nut_intermediate_a_bot,0.0,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1,approach_axis="z", lift_up_after_place = True)

    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("screw_ready", "c_bot")
    rospy.logwarn("The following sequence assumes that the nut tool m6 is being grasped by c_bot")
    #todo: pick up nut tool with c
    max_radius=0.005
    real_radius=0.
    rotation_axis="z"
    self.pick_nut_with_spiral_search(object_pose=nut_intermediate_c_bot,end_effector_link="c_bot_nut_tool_m6_tip_link")
    #do not make the grasp height of the above pick lower than 0.0!
    #to do: pick up nut with nut tool
    self.go_to_named_pose("screw_ready", "c_bot")
    return

  def pick_retainer_pin_washer(self):
    rospy.loginfo("============ Picking up the retainer pin washer 1 using a_bot ============")

    self.go_to_named_pose("home", "a_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_8"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.02

    self.pick("a_bot",pose0,-0.015,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)

    self.go_to_named_pose("home", "a_bot")
    return

  def pick_retainer_pin_washer_2(self):
    rospy.loginfo("============ Picking up the retainer pin washer 2 using a_bot ============")

    self.go_to_named_pose("home", "a_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_8"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.y = -0.015
    pose0.pose.position.z = 0.02


    self.pick("a_bot",pose0,-0.015,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)

    self.go_to_named_pose("home", "a_bot")
    return

  def place_retainer_pin_washer_2(self, robot_name = "a_bot", using_real_robot=True):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    
    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.x = -0.022
    intermediate_retainer_pin_tip.pose.position.y = 0.001
    intermediate_retainer_pin_tip.pose.position.z += 0.008

    self.place("a_bot",intermediate_retainer_pin_tip,-0.022,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0,approach_axis="x", lift_up_after_place = False)
    rospy.sleep(0.5)
    if using_real_robot is True:
      self.horizontal_spiral_motion("a_bot", .004)
      rospy.loginfo("doing spiral motion")

    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    intermediate_facing_sky.pose.position.x = 0.
    intermediate_facing_sky.pose.position.y = 0.
    intermediate_facing_sky.pose.position.z = -0.4
    self.go_to_pose_goal("b_bot", intermediate_facing_sky,speed=.31, move_lin = True)
    return

  def place_retainer_pin_washer_intermediate(self):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "workspace_center"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    intermediate_facing_sky.pose.position.x = -.13
    intermediate_facing_sky.pose.position.y = -.20

    #0.01637; 0.0021364; 0.012837
    self.place("a_bot",intermediate_facing_sky,0.0,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1,approach_axis="z", lift_up_after_place = False)


    self.go_to_named_pose("home", "a_bot")
    return

  def pick_retainer_pin_washer_intermediate(self):
    intermediate_facing_sky = geometry_msgs.msg.PoseStamped()
    intermediate_facing_sky.header.frame_id = "workspace_center"
    intermediate_facing_sky.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    intermediate_facing_sky.pose.position.x = -.13
    intermediate_facing_sky.pose.position.y = -.20

    #0.01637; 0.0021364; 0.012837
    self.pick("a_bot",intermediate_facing_sky, 0.0,
                                speed_fast = 0.31, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)

    self.go_to_named_pose("home", "a_bot")
    return

  def place_retainer_pin_washer_final(self):
    assembled_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_tip.header.frame_id = "assembled_assy_part_14_screw_tip"
    #tilted approach
    # assembled_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi*3./4.,0))
    #straight from above approach
    assembled_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi*2./4.,pi))
    assembled_retainer_pin_tip.pose.position.x = -0.003
    assembled_retainer_pin_tip.pose.position.y = 0
    assembled_retainer_pin_tip.pose.position.z = -0.001

    #0.01637; 0.0021364; 0.012837
    self.place("a_bot",assembled_retainer_pin_tip,-0.015,
                                speed_fast = 0.1, speed_slow = 0.05, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1,approach_axis="z", lift_up_after_place = True)


    self.go_to_named_pose("home", "a_bot")
    return

  def insert_retainer_pin_to_base(self):
    rospy.loginfo("============ inserting retainer pin to the base plate ============")
    self.go_to_named_pose("home", "a_bot")
    self.go_to_named_pose("home", "c_bot")
    
    assembled_retainer_pin_head = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_head.header.frame_id = "assembled_assy_part_14_screw_head"
    assembled_retainer_pin_head.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    assembled_retainer_pin_head.pose.position.x = 0.00
    assembled_retainer_pin_head.pose.position.y = -0.0024
    assembled_retainer_pin_head.pose.position.z = 0.01

    assembled_retainer_pin_head_final=copy.deepcopy(assembled_retainer_pin_head)
    assembled_retainer_pin_head_final.pose.position.x = 0.0052989
    assembled_retainer_pin_head_final.pose.position.y = -0.0022852
    assembled_retainer_pin_head_final.pose.position.z = 0.0032796

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
  
  def hold_idle_pulley_with_a_bot(self,using_real_robot=True):
    assembled_retainer_pin_head = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_head.header.frame_id = "assembled_assy_part_05_center"
    assembled_retainer_pin_head.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    assembled_retainer_pin_head.pose.position.x = 0
    assembled_retainer_pin_head.pose.position.y = -0.0024
    assembled_retainer_pin_head.pose.position.z = 0.005
    
    assembled_retainer_pin_head_approach = copy.deepcopy(assembled_retainer_pin_head)
    assembled_retainer_pin_head_approach.pose.position.z += 0.04
    if using_real_robot is True:
      self.precision_gripper_inner_open()
    self.go_to_pose_goal("a_bot", assembled_retainer_pin_head_approach, speed=.1, move_lin = True)
    self.go_to_pose_goal("a_bot", assembled_retainer_pin_head, speed=.01, move_lin = True)

    if using_real_robot is True:
      self.precision_gripper_inner_close(this_action_grasps_an_object = True)
    return
  
  def release_and_push_with_b_bot(self):
    x=raw_input("press enter to continue with the next part of the sequence")
    self.send_gripper_command("b_bot", "open")
    self.send_gripper_command("b_bot", "open")
    self.send_gripper_command("b_bot", "open")
    self.send_gripper_command("b_bot", "open")
    self.send_gripper_command("b_bot", "open")
    rospy.sleep(1.0)
    rospy.loginfo("Moving backwards. Did the gripper open? If yes, press enter.")
    raw_input()

    b_bot_going_back = geometry_msgs.msg.PoseStamped()
    b_bot_going_back.header.frame_id = "assembled_assy_part_14_screw_head"
    b_bot_going_back.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    b_bot_going_back.pose.position.x = -0.03
    b_bot_going_back.pose.position.y = -0.0024
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
    self.do_linear_push("b_bot", 5, wait = True)
    return

  def release_idle_pulley_from_a_bot(self, using_real_robot=True):
    x=raw_input("press enter to continue with the next part of the sequence")
    assembled_retainer_pin_head_retreat = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_head_retreat.header.frame_id = "assembled_assy_part_05_center"
    assembled_retainer_pin_head_retreat.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi/2))
    assembled_retainer_pin_head_retreat.pose.position.x = 0
    assembled_retainer_pin_head_retreat.pose.position.y = -0.0024
    assembled_retainer_pin_head_retreat.pose.position.z = 0.045
    if using_real_robot is True:
      self.precision_gripper_inner_open()
    self.go_to_pose_goal("a_bot", assembled_retainer_pin_head_retreat, speed=.1, move_lin = True)

    self.go_to_named_pose("home", "a_bot")
    return

  def fasten_retainer_pin_nut(self):
    rospy.loginfo("ToDo, fasten_retainer_pin_nut")
    assembled_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    assembled_retainer_pin_tip.header.frame_id = "assembled_assy_part_14_screw_tip"

    assembled_retainer_pin_tip.pose.position.x = 0.050309
    assembled_retainer_pin_tip.pose.position.y = -0.0051578
    assembled_retainer_pin_tip.pose.position.z = 0.040285
    assembled_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, pi, 0))
    # assembled_retainer_pin_tip.pose.orientation.x = 0.14067
    # assembled_retainer_pin_tip.pose.orientation.y = 0.99003
    # assembled_retainer_pin_tip.pose.orientation.z = 0.00022566
    # assembled_retainer_pin_tip.pose.orientation.w = 0.0078231

    assembled_retainer_pin_tip_2 = copy.deepcopy(assembled_retainer_pin_tip)
    assembled_retainer_pin_tip_2.pose.position.x = 0.01
    assembled_retainer_pin_tip_2.pose.position.y = 0.00
    assembled_retainer_pin_tip_2.pose.position.z = -0.007

    assembled_retainer_pin_tip_3 = copy.deepcopy(assembled_retainer_pin_tip)
    assembled_retainer_pin_tip_3.pose.position.x = -0.002
    assembled_retainer_pin_tip_3.pose.position.y = 0.00
    assembled_retainer_pin_tip_3.pose.position.z = -0.007

    self.go_to_named_pose("home", "c_bot")
    self.go_to_pose_goal("c_bot", assembled_retainer_pin_tip, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.go_to_pose_goal("c_bot", assembled_retainer_pin_tip_2, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")
    self.go_to_pose_goal("c_bot", assembled_retainer_pin_tip_3, speed=.1, move_lin = True, end_effector_link="c_bot_nut_tool_m6_tip_link")


  def put_on_belt(self):
    ps_b_pick_approach = geometry_msgs.msg.PoseStamped()
    ps_b_pick_approach.header.frame_id = "tray_1_partition_3"
    ps_b_pick_approach.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(pi/4, pi/2, 0))
    ps_b_pick_approach.pose.position.x = -.06
    ps_b_pick_approach.pose.position.y = -0.025
    ps_b_pick_approach.pose.position.z = 0.02
    ps_b_pick = copy.deepcopy(ps_b_pick_approach)
    ps_b_pick.pose.position.z = -0.015
    ps_b_above_belt_present = copy.deepcopy(ps_b_pick_approach)
    ps_b_above_belt_present.header.frame_id = "assembled_assy_part_11_front_hole"
    ps_b_above_belt_present.pose.position.x = 0.006
    ps_b_above_belt_present.pose.position.y = 0.056
    ps_b_above_belt_present.pose.position.z = 0.114
    ps_b_above_belt_present.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    ps_b_belt_present = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_present.pose.position.z = 0.03

    ps_a_belt_approach = copy.deepcopy(ps_b_above_belt_present)
    ps_a_belt_approach.pose.position.x = 0.0012
    ps_a_belt_approach.pose.position.y = 0.161
    ps_a_belt_approach.pose.position.z = 0.081
    ps_a_belt_approach.pose.orientation.x = -0.60423
    ps_a_belt_approach.pose.orientation.y = 0.63256
    ps_a_belt_approach.pose.orientation.z = 0.3505
    ps_a_belt_approach.pose.orientation.w = 0.33455
    ps_a_belt_grasp = copy.deepcopy(ps_a_belt_approach)
    ps_a_belt_grasp.pose.position.x = 0.015
    ps_a_belt_grasp.pose.position.y = 0.137
    ps_a_belt_place = copy.deepcopy(ps_a_belt_grasp)
    ps_a_belt_place.pose.position.x = 0.01482
    ps_a_belt_place.pose.position.y = 0.13746
    ps_a_belt_place.pose.position.z = 0.0082091
    ps_a_belt_place.pose.orientation.x = 0.60687
    ps_a_belt_place.pose.orientation.y = -0.58903
    ps_a_belt_place.pose.orientation.z = -0.38475
    ps_a_belt_place.pose.orientation.w = -0.36975
    
    # Possible extra hold poses
    # #temporary a_bot
    # 0.015942; 0.13972; -0.014877; 0.58922; -0.54298; -0.38705; -0.45627
    # #approach to temporary a_bot
    # 0.013052; 0.14083; 0.0017692; 0.58923; -0.54291; -0.38707; -0.45633

    ps_b_belt_put_0 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_0.pose.position.x = -0.023068
    ps_b_belt_put_0.pose.position.y = 0.022218
    ps_b_belt_put_0.pose.position.z = -0.0016302
    ps_b_belt_put_0.pose.orientation.x = -0.0011895
    ps_b_belt_put_0.pose.orientation.y = 0.0031101
    ps_b_belt_put_0.pose.orientation.z = 0.00065682
    ps_b_belt_put_0.pose.orientation.w = 0.99999
   
    ps_b_belt_put_1 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_1.pose.position.x = -0.029182
    ps_b_belt_put_1.pose.position.y = -0.010515
    ps_b_belt_put_1.pose.position.z = 0.010613

    ps_b_belt_put_2 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_2.pose.position.x = 0.0067392
    ps_b_belt_put_2.pose.position.y = -0.024667
    ps_b_belt_put_2.pose.position.z = 0.029542
    # ps_b_belt_put_2.pose.position.z = 0.012  # Manual tune through code
    ps_b_belt_put_2.pose.orientation.x =-0.026838
    ps_b_belt_put_2.pose.orientation.y = 0.20445
    ps_b_belt_put_2.pose.orientation.z = 0.021962
    ps_b_belt_put_2.pose.orientation.w = 0.97826

    ps_b_belt_put_3 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_3.pose.position.x = 0.010046
    ps_b_belt_put_3.pose.position.y = -0.025405
    ps_b_belt_put_3.pose.position.z = 0.01382
    ps_b_belt_put_3.pose.orientation.x = -0.047127
    ps_b_belt_put_3.pose.orientation.y = 0.27254
    ps_b_belt_put_3.pose.orientation.z = 0.096443
    ps_b_belt_put_3.pose.orientation.w = 0.95614

    ps_b_belt_put_4 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_4.pose.position.x = 0.0097681
    ps_b_belt_put_4.pose.position.y = -0.032955
    ps_b_belt_put_4.pose.position.z = -0.00017476
    ps_b_belt_put_4.pose.orientation.x = 0.13849
    ps_b_belt_put_4.pose.orientation.y = 0.28608
    ps_b_belt_put_4.pose.orientation.z = 0.042087
    ps_b_belt_put_4.pose.orientation.w = 0.94721

    ps_b_belt_put_5 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_5.pose.position.x = 0.0096936
    ps_b_belt_put_5.pose.position.y = -0.028125
    ps_b_belt_put_5.pose.position.z = -0.011783
    ps_b_belt_put_5.pose.orientation.x = 0.25341
    ps_b_belt_put_5.pose.orientation.y = 0.28907
    ps_b_belt_put_5.pose.orientation.z = 0.0068096
    ps_b_belt_put_5.pose.orientation.w = 0.92313
    
    ps_b_belt_put_6 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_6.pose.position.x = 0.0096176
    ps_b_belt_put_6.pose.position.y = -0.023353
    ps_b_belt_put_6.pose.position.z = -0.020149
    ps_b_belt_put_6.pose.orientation.x = 0.4333
    ps_b_belt_put_6.pose.orientation.y = 0.28455
    ps_b_belt_put_6.pose.orientation.z = -0.051186
    ps_b_belt_put_6.pose.orientation.w = 0.85362

    ps_b_belt_put_7 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_7.pose.position.x = 0.0095138
    ps_b_belt_put_7.pose.position.y = -0.018058
    ps_b_belt_put_7.pose.position.z = -0.023723
    ps_b_belt_put_7.pose.orientation.x = 0.47823
    ps_b_belt_put_7.pose.orientation.y = 0.28154
    ps_b_belt_put_7.pose.orientation.z = -0.066498
    ps_b_belt_put_7.pose.orientation.w = 0.82922

    ps_b_belt_put_8 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_8.pose.position.x = 0.0095106
    ps_b_belt_put_8.pose.position.y = -0.016251
    ps_b_belt_put_8.pose.position.z = -0.026674
    ps_b_belt_put_8.pose.orientation.x = 0.57396
    ps_b_belt_put_8.pose.orientation.y = 0.27155
    ps_b_belt_put_8.pose.orientation.z = -0.099617
    ps_b_belt_put_8.pose.orientation.w = 0.316609

    ps_b_belt_put_9 = copy.deepcopy(ps_b_above_belt_present)
    ps_b_belt_put_9.pose.position.x = 0.011757
    ps_b_belt_put_9.pose.position.y = -0.010226
    ps_b_belt_put_9.pose.position.z = -0.021166
    ps_b_belt_put_9.pose.orientation.x = 0.63966
    ps_b_belt_put_9.pose.orientation.y = 0.16901
    ps_b_belt_put_9.pose.orientation.z = 0.05053
    ps_b_belt_put_9.pose.orientation.w = 0.74814

    # ps_b_belt_put_10 = copy.deepcopy(ps_b_above_belt_present)
    # ps_b_belt_put_10.pose.position.x = 0.0095106
    # ps_b_belt_put_10.pose.position.y = -0.016251
    # ps_b_belt_put_10.pose.position.z = -0.026674
    # ps_b_belt_put_10.pose.orientation.x = 0.57396
    # ps_b_belt_put_10.pose.orientation.y = 0.27155
    # ps_b_belt_put_10.pose.orientation.z = -0.099617
    # ps_b_belt_put_10.pose.orientation.w = 0.76609

    ps_a_belt_put_0 = copy.deepcopy(ps_b_above_belt_present)
    ps_a_belt_put_0.pose.position.x = -0.1284726
    ps_a_belt_put_0.pose.position.y = 0.14147
    ps_a_belt_put_0.pose.position.z = 0.10941
    ps_a_belt_put_0.pose.orientation.x = 0.84672
    ps_a_belt_put_0.pose.orientation.y = -0.41555
    ps_a_belt_put_0.pose.orientation.z = -0.061494
    ps_a_belt_put_0.pose.orientation.w = -0.3265

    ps_a_belt_put_1 = copy.deepcopy(ps_b_above_belt_present)
    ps_a_belt_put_1.pose.position.x = 0.00037505
    ps_a_belt_put_1.pose.position.y = 0.051676
    ps_a_belt_put_1.pose.position.z = 0.0011139
    ps_a_belt_put_1.pose.orientation.x = 0.99502
    ps_a_belt_put_1.pose.orientation.y = -0.089389
    ps_a_belt_put_1.pose.orientation.z = -0.042746
    ps_a_belt_put_1.pose.orientation.w = -0.010752

    ps_a_belt_put_2 = copy.deepcopy(ps_b_above_belt_present)
    ps_a_belt_put_2.pose.position.x = 0.00037505
    ps_a_belt_put_2.pose.position.y = 0.051676
    ps_a_belt_put_2.pose.position.z = 0.0011139
    ps_a_belt_put_2.pose.orientation.x = 0.99502
    ps_a_belt_put_2.pose.orientation.y = -0.089389
    ps_a_belt_put_2.pose.orientation.z = -0.042746
    ps_a_belt_put_2.pose.orientation.w = -0.010752

    ps_a_belt_put_3 = copy.deepcopy(ps_b_above_belt_present)
    ps_a_belt_put_3.pose.position.x = 0.029154
    ps_a_belt_put_3.pose.position.y = 0.039011
    ps_a_belt_put_3.pose.position.z = -0.024386
    ps_a_belt_put_3.pose.orientation.x = 0.99248
    ps_a_belt_put_3.pose.orientation.y = -0.11621
    ps_a_belt_put_3.pose.orientation.z = -0.0092272
    ps_a_belt_put_3.pose.orientation.w = -0.037367

    # ==========
    
    self.send_gripper_command("b_bot", .02)
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")
        
    self.move_lin("b_bot", ps_b_pick_approach, 1.0)
    rospy.sleep(1)
    self.move_lin("b_bot", ps_b_pick, 0.1)
    rospy.sleep(1)
    self.send_gripper_command("b_bot", "close")
    self.move_lin("b_bot", ps_b_pick_approach, 1.0)
    rospy.sleep(1)
    
    self.move_lin("b_bot", ps_b_above_belt_present, 1.0)
    rospy.sleep(1)
    self.move_lin("b_bot", ps_b_belt_present, 1.0)
    rospy.sleep(1)
    self.move_lin("a_bot", ps_a_belt_approach, 1.0)
    rospy.sleep(1)
    self.move_lin("a_bot", ps_a_belt_grasp, 1.0)
    rospy.sleep(1)
    self.send_gripper_command("precision_gripper_inner", "close")
    self.move_lin("a_bot", ps_a_belt_place, .03)
    rospy.sleep(1)
    # self.send_gripper_command("precision_gripper_inner", "close")
    rospy.loginfo("Press enter.")
    raw_input()
    self.move_lin("b_bot", ps_b_belt_put_0, .03)
    rospy.sleep(.5)
    self.move_lin("b_bot", ps_b_belt_put_1, .03)
    rospy.sleep(.5)
    self.move_lin("b_bot", ps_b_belt_put_2, .03)
    rospy.sleep(.1)
    self.move_lin("b_bot", ps_b_belt_put_3, .03)
    rospy.sleep(.1)
    self.move_lin("b_bot", ps_b_belt_put_4, .03)
    rospy.sleep(.1)
    self.move_lin("b_bot", ps_b_belt_put_5, .03)
    rospy.sleep(.1)
    self.move_lin("b_bot", ps_b_belt_put_6, .03)
    rospy.sleep(.1)
    self.move_lin("b_bot", ps_b_belt_put_7, .03)
    rospy.sleep(.1)
    self.move_lin("b_bot", ps_b_belt_put_8, .03)
    rospy.sleep(.1)
    # self.move_lin("b_bot", ps_b_belt_put_9, .03)
    # rospy.sleep(.1)
    rospy.loginfo("Press enter.")
    raw_input()

    self.move_lin("a_bot", ps_a_belt_approach, .1)
    rospy.sleep(1)
    self.move_lin("a_bot", ps_a_belt_put_0, .1)
    rospy.sleep(1)
    self.move_lin("a_bot", ps_a_belt_put_1, .03)
    rospy.sleep(1)
    self.move_lin("a_bot", ps_a_belt_put_2, .03)
    rospy.sleep(1)
    self.move_lin("a_bot", ps_a_belt_put_3, .03)
    rospy.sleep(1)
    self.send_gripper_command("b_bot", 0.015)

  def pick_shaft_spacer(self):
    rospy.loginfo("============ Picking up a retainer pin using b_bot ============")

    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_4"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.0

    self.do_pick_action("b_bot", pose0, z_axis_rotation = 0.0, use_complex_planning = False)
    return

  def insert_shaft_spacer(self):
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_11_front_hole"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pre_insertion.pose.position.x = -0.03
    pre_insertion.pose.position.y = -0.002
    pre_insertion.pose.position.z = 0.008

    self.go_to_pose_goal("b_bot", pre_insertion,speed=.2, move_lin = True)

    print ("Please ask Felix what the differences between the two functions below are")
    # self.do_insertion(robot_name="b_bot")
    # self.do_insert_action(active_robot_name="b_bot", passive_robot_name = "c_bot")
    return

  def pick_motor(self):
    self.go_to_named_pose("home", "b_bot")
    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_4"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    print pose0
    self.pick("b_bot",pose0,0.05,
                                speed_fast = 0.7, speed_slow = 0.05, gripper_command="xxx",
                                approach_height = 0.13)
    self.go_to_named_pose("home", "b_bot")
    return
  def handover_motor(self):
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
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_04_inserted_13"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, -pi/2, 0))
    pre_insertion.pose.position.x = -0.05

    self.go_to_pose_goal("c_bot", pre_insertion, speed=.3, move_lin = True)
    print "inserting using linear_push in Y negative direction, please ask Felix how to call linear push for different axis using his o2as_skills server"
    # impedance control may not be necessary in this case, it is not as difficult
    return
  
  def fasten_motor_screw_1(self):#for picking up and fastening a screw. need to expand this for 6 screws. I think just make 6 different functions.
    pose1 = geometry_msgs.msg.PoseStamped()
    pose1.header.frame_id = "assembled_assy_part_02_motor_screw_hole_1"
    pose1.pose.orientation.w =   0.707
    pose1.pose.orientation.x =  -0.707
    pose1.pose.orientation.y = 0
    pose1.pose.orientation.z = 0
    # pose1.pose.position.y = 0.
    # pose1.pose.position.x = 0.
    # pose1.pose.position.z = 0.
    
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    # self.pick_screw("b_bot", screw_size=3, screw_number=1) # I commented this because this takes a long time in simulation
    self.go_to_named_pose("screw_ready", "b_bot")
    self.go_to_pose_goal("b_bot", pose1, speed=0.3,end_effector_link="b_bot_screw_tool_m4_tip_link", move_lin=True)
    # todo: add fastening action
    return

  def pick_motor_pulley(self):
    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_6"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.07

    self.do_pick_action("b_bot", pose0, z_axis_rotation = 0.0, use_complex_planning = False)
    return

  def insert_motor_pulley(self):
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_04_tip"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    pre_insertion.pose.position.x = 0.04
    self.go_to_pose_goal("b_bot", pre_insertion, speed=.3, move_lin = True)

  def pick_bearing(self):
    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_2"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.07

    self.do_pick_action("b_bot", pose0, z_axis_rotation = 0.0, use_complex_planning = False)
    return

  def insert_bearing(self):
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_11_front_hole"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pre_insertion.pose.position.x = -0.04
    self.go_to_pose_goal("b_bot", pre_insertion, speed=.3, move_lin = True)

  def pick_clamping_pulley(self):
    self.go_to_named_pose("home", "b_bot")

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_1_partition_1"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.07

    self.do_pick_action("b_bot", pose0, z_axis_rotation = 0.0, use_complex_planning = False)
    return

  def insert_clamping_pulley(self):
    pre_insertion = geometry_msgs.msg.PoseStamped()
    pre_insertion.header.frame_id = "assembled_assy_part_11_front_hole"
    pre_insertion.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pre_insertion.pose.position.x = -0.04
    self.go_to_pose_goal("b_bot", pre_insertion, speed=.3, move_lin = True)

if __name__ == '__main__':
  try:
    print ("Please refer to this page for details of each subtask. https://docs.google.com/spreadsheets/d/1Os2CfH80A7vzj6temt5L8BYpLvHKBzWT0dVuTvpx5Mk/edit#gid=1216221803")
    print ("to run the function called 'pick' and 'place' in simulation correctly, change \"easy_pick_only_inner\" to something different like \"easy_pick_only_innerX\" in line 83, 101, 136")
    print ("if the sequence is stuck at 'opening inner gripper' or 'closing inner gripper', just comment out the line that sends commands to precision gripper")
    assy = AssemblyClass()
    assy.set_up_item_parameters()
    # ========================= Sending robots to home position ======================================
    # assy.go_to_named_pose("home", "c_bot")
    # assy.go_to_named_pose("home", "b_bot")
    # assy.go_to_named_pose("home", "a_bot")

    # ================== leftover of old stuff written by Felix =======================
    # assy.handover_demo()
    # assy.insertion_demo()
    # assy.belt_demo()
    
    # =============================== SUBTASK F ======================================================
    # assy.place_plate_2()
    print ("todo: add screw picking and fastening sequence")

    # ============= SUBTASK A (picking and inserting and fastening the motor shaft) =======================
    # assy.pick_motor()
    # assy.adjust_centering()
    # assy.handover_motor()
    print ("todo: pick up m3 tool using b bot, replace this print with the equip function")
    # assy.insert_motor() # Joshua thinks this may be possible to do without impedance control, otherwise use insertion script in Y negative direction
    # assy.go_to_named_pose("screw_ready", "b_bot")
    # assy.fasten_motor_screw_1() # please ask Felix about the pick_screw function, I am not sure how he defined it
    print ("todo: copy and paste fasten_motor_screw_1 to add fasten_motor_screw_2 ~ 6")
    

    # ================================= SUBTASK B (motor pulley) ===========================================
    # assy.pick_motor_pulley()
    # assy.insert_motor_pulley()
    print ("todo: fasten motor pulley")

    # ===== SUBTASK G (Placing and fastening the output (larger) plate, for idle pulley set and clamping pulley set) =========
    # assy.place_plate_3_and_screw()
    print ("todo: add screw picking and fastening sequence for the second screw")


    # ==== SUBTASK C (clamping pulley set, everything but inserting and fastening clamping pulley) =================
    # assy.pick_bearing()
    # assy.insert_bearing()
    print ("todo: pick up shaft, pick up cap, insert the cap, fasten the cap, insert using impedance Y negative direction (using b_bot)")
    print ("todo: make the gripper not fully open while approaching for picking, may be important for motor pulley and clamping_shaft_spacer")
    # assy.pick_shaft_spacer()
    # assy.insert_shaft_spacer()


    # ============= SUBTASK D (inserting and fastening clamping pulley) =======================
    # assy.pick_clamping_pulley()
    # assy.insert_clamping_pulley()
    print ("todo: fastening clamping pulley")

    # ======================== SUBTASK E (The idler pin) ============================================
    # assy.pick_retainer_pin()
    # assy.adjust_centering()
    # assy.rotate_hand_facing_the_sky()
    # assy.pick_idle_pulley()
    # assy.place_idle_pulley(using_real_robot=False)
    # assy.pick_retainer_pin_spacer()
    # assy.place_retainer_pin_spacer(using_real_robot=False)
    # assy.pick_retainer_pin_washer_2()
    # assy.place_retainer_pin_washer_2(using_real_robot=False)
    # assy.pick_retainer_pin_nut()
    print ("todo: pick nut with tool, fasten retainer pin nut")
    print ("todo: add equip and unequip action into the codes")
    # assy.place_retainer_pin_nut_and_pick_with_tool()
    # assy.pick_retainer_pin_washer()
    # assy.place_retainer_pin_washer_intermediate()
    # assy.insert_retainer_pin_to_base()
    # assy.hold_idle_pulley_with_a_bot()
    # assy.release_and_push_with_b_bot()
    # assy.release_idle_pulley_from_a_bot()
    # assy.pick_retainer_pin_washer_intermediate()
    # assy.place_retainer_pin_washer_final()
    # assy.fasten_retainer_pin_nut()

    # ====================================== SUBTASK H ===============================================
    # assy.put_on_belt()

    # ====================== SAMPLE CODE ======================-

    #  ============ Equip then unequip tool with c_bot ============
    # assy.go_to_named_pose("home", "c_bot")
    # assy.do_change_tool_action("c_bot", screw_size=4, equip=True)    
    # assy.go_to_named_pose("screw_ready", "c_bot")
    # assy.do_change_tool_action("c_bot", screw_size=4, equip=False)

    # ============= Equip and unequip tool with b_bot ==========
    # while equiping and unequiping tool with b_bot, c_bot needs to be in "back" pose
    # assy.go_to_named_pose("back", "c_bot")
    # assy.do_change_tool_action("b_bot", screw_size=4, equip=True)

    print("=========== PLEASE ASK FELIX ABOUT THIS ===============")
    # print(assy.planning_scene_interface.get_known_object_names())
    # assy.planning_scene_interface.disallow_collisions("screw_tool_m4")
    # raw_input()
    # assy.planning_scene_interface.allow_collisions("screw_tool_m4")
  except rospy.ROSInterruptException:
    pass
