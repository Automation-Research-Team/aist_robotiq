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
  def pick(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.03):
    #initial gripper_setup
    rospy.loginfo("Going above object to pick")
    object_pose.pose.position.z = approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner":
      self.precision_gripper_inner_close()
    else: 
      rospy.logerr("No gripper command was set")

    rospy.loginfo("Moving down to object")
    object_pose.pose.position.z = grasp_height
    rospy.loginfo(grasp_height)
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True)

    # W = raw_input("waiting for the gripper")
    #gripper close
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_close(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="easy_pick_only_inner":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
    rospy.sleep(2)
    rospy.loginfo("Going back up")
    object_pose.pose.position.z = (approach_height)
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

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
    elif gripper_command=="easy_pick_only_inner":
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
    self.go_to_named_pose("screw_pick_ready", "b_bot")
    self.go_to_named_pose("home", "a_bot")

    # Pick up screw from tray
    # Call the pick action
    goal = o2as_msgs.msg.pickGoal()
    goal.robot_name = "b_bot"
    goal.tool_name = "screw_tool"
    goal.screw_size = 4
    pscrew = geometry_msgs.msg.PoseStamped()
    pscrew.header.frame_id = "tray_2_screw_m4_1" # The top corner of the big plate
    pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(-pi/2, 0,0))
    goal.item_pose = pscrew
    rospy.loginfo("Sending pick action goal")
    rospy.loginfo(goal)

    self.pick_client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    self.pick_client.wait_for_result()
    rospy.loginfo("Getting result")
    self.pick_client.get_result()

    p_screw_rest = geometry_msgs.msg.PoseStamped()
    p_screw_rest.header.frame_id = "screw_tool_m6_helper_link" # The top corner of the big plate
    p_screw_rest.pose.position.y = .15
    p_screw_rest.pose.position.z = .25
    p_screw_rest.pose.orientation.w = 1.0
    self.move_lin("b_bot", p_screw_rest, 0.05)

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
    ps_place = ps_pickup
    ps_place.header.frame_id = "assembled_assy_part_03_pulley_ridge_bottom"
    ps_place.pose.position.z += .001
    ps_move_away = ps_place
    ps_move_away.pose.position.y -= .06
    
    self.move_lin("c_bot", ps_approach, 1.0)

    self.move_lin("c_bot", ps_pickup, 1.0)
    self.send_gripper_command("c_bot", "close")
    # raw_input() # Uncomment this to draw the contour as it is grasped

    self.move_lin("c_bot", ps_high, 1.0)

    # Deliver the item to its assembled position
    ps_place.pose.position.z += .005
    self.move_lin("c_bot", ps_place, .3)
    ps_place.pose.position.z -= .005
    self.move_lin("c_bot", ps_place, .02)
    self.send_gripper_command("c_bot", 0.008)

    # Move out of the way
    self.move_lin("c_bot", ps_move_away, .3)
    self.go_to_named_pose("back", "c_bot")
    
    ###### ==========
    # Move b_bot to the hole and screw
    self.go_to_named_pose("back", "b_bot")
    pscrew = geometry_msgs.msg.PoseStamped()
    pscrew.header.frame_id = "assembled_assy_part_03_bottom_screw_hole_1" # The top corner of the big plate
    pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, 0,0))
    # pscrew.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(pi/2, 0,0))
    self.do_screw_action("b_bot", pscrew, screw_height = 0.02, screw_size = 4)

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
    pose0.pose.position.z = 0.008

    # if robot_name=="b_bot":
    #   self.go_to_pose_goal(robot_name, pose0,speed=.05, move_lin = True)
    #   # pose0.pose.position.x = -.01
    #   rospy.sleep(1.0)
    # pose0.pose.position.z =0
    self.send_gripper_command(gripper="b_bot",command = 0.04)
    self.do_pick_action(robot_name, pose0, z_axis_rotation = 0.0, use_complex_planning = False)
    
    #self.go_to_named_pose("home", "b_bot")
    self.groups["b_bot"].set_joint_value_target([1.75, -1.24, 1.28, -0.00, 0.23, -1.55])
    self.groups["b_bot"].set_max_velocity_scaling_factor(.3)
    self.groups["b_bot"].go(wait=True)
    self.groups["b_bot"].stop()

    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_head"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    
    self.go_to_pose_goal(robot_name, intermediate_retainer_pin_tip,speed=.2, move_lin = True)
    intermediate_retainer_pin_tip.pose.position.z = -0.4
    self.go_to_pose_goal(robot_name, intermediate_retainer_pin_tip,speed=.2, move_lin = True)
    return

  def pick_and_place_retainer_pin_spacer(self, robot_name = "a_bot"):
    rospy.loginfo("============ Picking up a retainer pin spacer using a_bot ============")

    self.go_to_named_pose("home", robot_name)

    pose0 = geometry_msgs.msg.PoseStamped()
    pose0.header.frame_id = "tray_2_partition_3"
    pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    pose0.pose.position.x = 0
    pose0.pose.position.z = 0.02

    self.pick("a_bot",pose0,-0.01,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                approach_height = 0.1)

    self.go_to_named_pose("home", robot_name)

    intermediate_retainer_pin_tip = geometry_msgs.msg.PoseStamped()
    intermediate_retainer_pin_tip.header.frame_id = "intermediate_assy_part_14_screw_tip"
    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

    self.go_to_pose_goal("b_bot", intermediate_retainer_pin_tip,speed=.2, move_lin = True)

    intermediate_retainer_pin_tip.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi, 0))
    intermediate_retainer_pin_tip.pose.position.x += 0.01637
    intermediate_retainer_pin_tip.pose.position.y += 0.0021364
    intermediate_retainer_pin_tip.pose.position.z += 0.012837

    #0.01637; 0.0021364; 0.012837
    self.place("a_bot",intermediate_retainer_pin_tip,0.01637,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                approach_height = 0.03,approach_axis="x", lift_up_after_place = False)
    self.horizontal_spiral_motion("a_bot", .004, intermediate_retainer_pin_tip)
    rospy.loginfo("doing spiral motion")


    # pin_held_up = geometry_msgs.msg.PoseStamped()
    # pin_held_up.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, pi/2))
    # pin_held_up.pose.position.x = 0.07
    # pin_held_up.pose.position.y = 0.20
    # pin_held_up.pose.position.z = .47

    # if robot_name=="a_bot":
      # self.go_to_pose_goal(robot_name, pose0,speed=.05, move_lin = True)
      # pose0.pose.position.x = -.01
      # rospy.sleep(1.0)
    # pose0.pose.position.z =0
    # self.send_gripper_command(gripper="b_bot",command = 0.04)
    # self.do_pick_action(robot_name, pose0, z_axis_rotation = 0.0, gripper_command = "complex_pick_from_inside")
    return


if __name__ == '__main__':
  try:
    assy = AssemblyClass()
    assy.set_up_item_parameters()
    assy.go_to_named_pose("home", "c_bot")
    # assy.go_to_named_pose("home", "b_bot")
    assy.go_to_named_pose("home", "a_bot")
    # assy.handover_demo()
    # assy.insertion_demo()
    # assy.belt_demo()
    # assy.place_plate_3_and_screw_demo()
    assy.place_plate_3_and_screw()
    # assy.place_plate_2()
    assy.pick_retainer_pin()
    assy.pick_and_place_retainer_pin_spacer()
    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
