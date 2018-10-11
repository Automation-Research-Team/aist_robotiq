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

import ur_modern_driver.msg


def is_program_running(topic_namespace = ""):
  """Checks if a program is running on the UR"""
  msg = rospy.wait_for_message(topic_namespace + "/ur_driver/robot_mode_state", ur_modern_driver.msg.RobotModeDataMsg)
  if msg:
    return msg.is_program_running
  else:
    rospy.logerr("No message received from the robot. Is everything running? Is the namespace entered correctly with a leading slash?")
    return False
    # throw()

def wait_for_UR_program(topic_namespace = "", timeout_duration = rospy.Duration.from_sec(20.0)):
  rospy.logdebug("Waiting for UR program to finish.")
  # Only run this after sending custom URScripts and not the regular motion commands, or this call will not terminate before the timeout.
  rospy.sleep(1.0)
  t_start = rospy.Time.now()
  time_passed = rospy.Time.now() - t_start
  while is_program_running(topic_namespace):
    rospy.sleep(.05)
    time_passed = rospy.Time.now() - t_start
    if time_passed > timeout_duration:
      rospy.loginfo("Timeout reached.")
      return False
  rospy.logdebug("UR Program has terminated.")
  return True

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
    self.use_real_robot = rospy.get_param("use_real_robot")
    self.force_ur_script_linear_motion = False
    self.force_moveit_linear_motion = False

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('assembly_example', anonymous=False)

    self.robots = moveit_commander.RobotCommander()
    self.planning_scene_interface = moveit_commander.PlanningSceneInterface()
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
    self.change_tool_client = actionlib.SimpleActionClient('/o2as_skills/changeTool', o2as_msgs.msg.changeToolAction)

    self.fastening_tool_client = actionlib.SimpleActionClient('/o2as_fastening_tools/fastener_gripper_control_action', o2as_msgs.msg.FastenerGripperControlAction)
    self.nut_peg_tool_client = actionlib.SimpleActionClient('/nut_tools_action', o2as_msgs.msg.ToolsCommandAction)

    self.urscript_client = rospy.ServiceProxy('/o2as_skills/sendScriptToUR', o2as_msgs.srv.sendScriptToUR)
    self.goToNamedPose_client = rospy.ServiceProxy('/o2as_skills/goToNamedPose', o2as_msgs.srv.goToNamedPose)
    self.publishMarker_client = rospy.ServiceProxy('/o2as_skills/publishMarker', o2as_msgs.srv.publishMarker)
    self.toggleCollisions_client = rospy.ServiceProxy('/o2as_skills/toggleCollisions', std_srvs.srv.SetBool)

    rospy.sleep(.5)
    rospy.loginfo("Finished initializing class")
    
  ############## ------ Internal functions (and convenience functions)

  def publish_marker(self, pose_stamped, marker_type):
    req = o2as_msgs.srv.publishMarkerRequest()
    req.marker_pose = pose_stamped
    req.marker_type = marker_type
    self.publishMarker_client.call(req)
    return True

  def go_to_pose_goal(self, group_name, pose_goal_stamped, speed = 1.0, acceleration = 0.0, high_precision = False, 
                      end_effector_link = "", move_lin = True):
    if move_lin:
      return self.move_lin(group_name, pose_goal_stamped, speed, acceleration, end_effector_link)
    self.publish_marker(pose_goal_stamped, "pose")
    group = self.groups[group_name]
    
    if not end_effector_link:
      if group_name == "c_bot":
        end_effector_link = "c_bot_robotiq_85_tip_link"
      elif group_name == "b_bot":
        end_effector_link = "b_bot_robotiq_85_tip_link"
      elif group_name == "a_bot":
        end_effector_link = "a_bot_gripper_tip_link"
    group.set_end_effector_link(end_effector_link)
    
    group.set_pose_target(pose_goal_stamped)
    rospy.logdebug("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)

    if high_precision:
      group.set_goal_tolerance(.000001)
      group.set_planning_time(10)

    move_success = group.go(wait=True)
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    
    # Reset the precision
    if high_precision:
      group.set_goal_tolerance(.0001) 
      group.set_planning_time(3) 

    current_pose = group.get_current_pose().pose
    return all_close(pose_goal_stamped.pose, current_pose, 0.01), move_success

  def transformTargetPoseFromTipLinkToEE(self, ps, robot_name, end_effector_link):
    rospy.logdebug("Received pose to transform to EE link:")
    rospy.logdebug(str(ps.pose.position.x) + ", " + str(ps.pose.position.y)  + ", " + str(ps.pose.position.z))
    rospy.logdebug(str(ps.pose.orientation.x) + ", " + str(ps.pose.orientation.y)  + ", " + str(ps.pose.orientation.z)  + ", " + str(ps.pose.orientation.w))

    t = self.listener.lookupTransform(end_effector_link, robot_name + "_tool0", rospy.Time())

    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = ps.header.frame_id
    m.child_frame_id = "temp_goal_pose__"
    m.transform.translation.x = ps.pose.position.x
    m.transform.translation.y = ps.pose.position.y
    m.transform.translation.z = ps.pose.position.z
    m.transform.rotation.x = ps.pose.orientation.x
    m.transform.rotation.y = ps.pose.orientation.y
    m.transform.rotation.z = ps.pose.orientation.z
    m.transform.rotation.w = ps.pose.orientation.w
    self.listener.setTransform(m)

    m.header.frame_id = "temp_goal_pose__"
    m.child_frame_id = "temp_wrist_pose__"
    m.transform.translation.x = t[0][0]
    m.transform.translation.y = t[0][1]
    m.transform.translation.z = t[0][2]
    m.transform.rotation.x = t[1][0]
    m.transform.rotation.y = t[1][1]
    m.transform.rotation.z = t[1][2]
    m.transform.rotation.w = t[1][3]
    self.listener.setTransform(m)

    ps_wrist = geometry_msgs.msg.PoseStamped()
    ps_wrist.header.frame_id = "temp_wrist_pose__"
    ps_wrist.pose.orientation.w = 1.0

    ps_new = self.listener.transformPose(ps.header.frame_id, ps_wrist)

    rospy.logdebug("New pose:")
    rospy.logdebug(str(ps_new.pose.position.x) + ", " + str(ps_new.pose.position.y)  + ", " + str(ps_new.pose.position.z))
    rospy.logdebug(str(ps_new.pose.orientation.x) + ", " + str(ps_new.pose.orientation.y)  + ", " + str(ps_new.pose.orientation.z)  + ", " + str(ps_new.pose.orientation.w))

    return ps_new

  def move_lin(self, group_name, pose_goal_stamped, speed = 1.0, acceleration = 0.0, end_effector_link = ""):
    self.publish_marker(pose_goal_stamped, "pose")

    if not end_effector_link:
      if group_name == "c_bot":
        end_effector_link = "c_bot_robotiq_85_tip_link"
      elif group_name == "b_bot":
        end_effector_link = "b_bot_robotiq_85_tip_link"
      elif group_name == "a_bot":
        end_effector_link = "a_bot_gripper_tip_link"

    if self.force_ur_script_linear_motion or self.use_real_robot:
      if not self.force_moveit_linear_motion:
        rospy.logdebug("Real robot is being used. Send linear motion to robot controller directly via URScript.")
        req = o2as_msgs.srv.sendScriptToURRequest()
        req.program_id = "lin_move"
        req.robot_name = group_name
        req.target_pose = self.transformTargetPoseFromTipLinkToEE(pose_goal_stamped, group_name, end_effector_link)
        req.velocity = speed
        req.acceleration = acceleration
        res = self.urscript_client.call(req)
        wait_for_UR_program("/" + group_name +"_controller", rospy.Duration.from_sec(30.0))
        return res.success

    group = self.groups[group_name]
      
    group.set_end_effector_link(end_effector_link)
    group.set_pose_target(pose_goal_stamped)
    rospy.logdebug("Setting velocity scaling to " + str(speed))
    group.set_max_velocity_scaling_factor(speed)
    

    # FIXME: At the start of the program, get_current_pose() did not return the correct value. Should be a bug report.
    waypoints = []
    ### The current pose is not added anymore, because it causes a bug in Gazebo, and it is not necessary.
    # wpose1 = group.get_current_pose().pose
    # # rospy.loginfo("Wpose1:")
    # # rospy.loginfo(wpose1)
    # rospy.sleep(.05)
    # wpose2 = group.get_current_pose().pose
    # # rospy.loginfo("Wpose2:")
    # # rospy.loginfo(wpose2)
    # waypoints.append(wpose2)
    pose_goal_world = self.listener.transformPose("world", pose_goal_stamped).pose
    waypoints.append(pose_goal_world)
    (plan, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
    rospy.loginfo("Compute cartesian path succeeded with " + str(fraction*100) + "%")
    plan = group.retime_trajectory(self.robots.get_current_state(), plan, speed)

    plan_success = group.execute(plan, wait=True)
    group.stop()
    group.clear_pose_targets()

    current_pose = group.get_current_pose().pose
    return plan_success

  def move_joints(self, group_name, joint_pose_goal, speed = 1.0, acceleration = 0.0, force_ur_script=False, force_moveit=False):
    if force_ur_script or self.use_real_robot:
      if not force_moveit:
        rospy.logdebug("Real robot is being used. Send joint command to robot controller directly via URScript.") 
        req = o2as_msgs.srv.sendScriptToURRequest()
        req.program_id = "move_j"
        req.robot_name = group_name
        req.joint_positions = joint_pose_goal
        req.velocity = speed
        req.acceleration = acceleration
        res = self.urscript_client.call(req)
        wait_for_UR_program("/" + group_name +"_controller", rospy.Duration.from_sec(10.0))
        return res.success

    self.groups[group_name].set_joint_value_target(joint_pose_goal)
    self.groups[group_name].set_max_velocity_scaling_factor(speed)
    return self.groups[group_name].go(wait=True)

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

  def horizontal_spiral_motion(self, robot_name, max_radius, radius_increment = .001, speed = 0.02, spiral_axis="Z"):
    rospy.loginfo("Performing horizontal spiral motion " + str(speed))
    req = o2as_msgs.srv.sendScriptToURRequest()
    req.program_id = "spiral_motion"
    req.robot_name = robot_name
    req.max_radius = max_radius
    req.radius_increment = radius_increment    
    req.velocity = speed
    req.spiral_axis = spiral_axis
    res = self.urscript_client.call(req)
    wait_for_UR_program("/" + robot_name +"_controller", rospy.Duration.from_sec(10.0))
    return res.success
    # =====

    # group = self.groups[robot_name]
    # rospy.loginfo("Performing horizontal spiral motion " + str(speed))
    # rospy.loginfo("Setting velocity scaling to " + str(speed))
    # group.set_max_velocity_scaling_factor(speed)
    # # Modified code from Robotiq spiral search
    # theta_incr = 30
    # radius_inc_set = radius_increment / (360 / theta_incr)
    # r=0.0003  #Start radius
    # theta=0
    # RealRadius=0
    
    # # ==== MISBEHAVING VERSION (see https://answers.ros.org/question/300978/movegroupcommander-get_current_pose-returns-incorrect-result-when-using-real-robot/ )
    # # start_pos_bugged = group.get_current_pose() 
    # # ==== WORKING VERSION:
    # gripper_pos = geometry_msgs.msg.PoseStamped()
    # gripper_pos.header.frame_id = "a_bot_gripper_tip_link"
    # gripper_pos.pose.orientation.w = 1.0
    # start_pos = self.listener.transformPose("world", gripper_pos)

    # next_pos = start_pos
    # while RealRadius <= max_radius and not rospy.is_shutdown():
    #     #By default, the Spiral_Search function will maintain contact between both mating parts at all times
    #     theta=theta+theta_incr
    #     x=cos(radians(theta))*r
    #     y=sin(radians(theta))*r
    #     next_pos.pose.position.x = start_pos.pose.position.x + x
    #     next_pos.pose.position.y = start_pos.pose.position.y + y
    #     r=r + radius_inc_set
    #     RealRadius = sqrt(pow(x,2)+pow(y,2))
    #     self.go_to_pose_goal(robot_name, next_pos)
    #     rospy.sleep(0.1)
    # # -------------
    # return True


  def go_to_named_pose(self, pose_name, robot_name, speed = 0.5, acceleration = 0.0, force_ur_script=False):
    # pose_name should be "home", "back" etc.
    if force_ur_script and self.use_real_robot:
      # joint_pose = self.groups[robot_name].get_joint_value_target() # This works only with a_bot. Bug?
      d = self.groups[robot_name].get_named_target_values(pose_name)
      joint_pose = [d[robot_name+"_shoulder_pan_joint"], 
                    d[robot_name+"_shoulder_lift_joint"],
                    d[robot_name+"_elbow_joint"],
                    d[robot_name+"_wrist_1_joint"],
                    d[robot_name+"_wrist_2_joint"],
                    d[robot_name+"_wrist_3_joint"]]
      self.move_joints(robot_name, joint_pose, speed, acceleration, force_ur_script=force_ur_script)
    if speed > 1.0:
      speed = 1.0
    self.groups[robot_name].set_named_target(pose_name)
    rospy.logdebug("Setting velocity scaling to " + str(speed))
    self.groups[robot_name].set_max_velocity_scaling_factor(speed)
    self.groups[robot_name].go(wait=True)
    # self.groups[robot_name].stop()
    self.groups[robot_name].clear_pose_targets()
    return True

  ######

  def pick(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, special_pick = False):
    #self.publish_marker(object_pose, "pick_pose")
    #initial gripper_setup
    rospy.loginfo("Going above object to pick")
    rospy.logdebug("Approach height 0: " + str(approach_height))
    object_pose.pose.position.z += approach_height
    rospy.logdebug("Height 1: " + str(object_pose.pose.position.z))
    if special_pick == True:
      object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi, pi*45/180, pi/2))
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast, move_lin=True)
    object_pose.pose.position.z -= approach_height
    rospy.logdebug("Height 2: " + str(object_pose.pose.position.z))
    self.publish_marker(object_pose, "place_pose")

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
    object_pose.pose.position.z += grasp_height
    rospy.logdebug("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True, move_lin=True)
    object_pose.pose.position.z -= grasp_height

    # W = raw_input("waiting for the gripper")
    #gripper close
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_inner_close(this_action_grasps_an_object = True)
      self.precision_gripper_outer_close()
    elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
      self.precision_gripper_inner_open(this_action_grasps_an_object = True)
    elif gripper_command=="easy_pick_outside_only_inner" or gripper_command=="inner_gripper_from_outside":
      self.precision_gripper_inner_close()
    elif gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="close")

    # if special_pick == True:
    #   object_pose.pose.orientation = self.downward_orientation
    rospy.sleep(1.0)
    rospy.loginfo("Going back up")
    object_pose.pose.position.z += approach_height
    rospy.loginfo("Going to height " + str(object_pose.pose.position.z))
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast, move_lin=True)
    object_pose.pose.position.z -= approach_height
    return True

  ######

  def place(self,robotname, object_pose, place_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, lift_up_after_place = True):
    #self.publish_marker(object_pose, "place_pose")
    rospy.loginfo("Going above place target")
    object_pose.pose.position.z += approach_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast, move_lin=True)
    object_pose.pose.position.z -= approach_height

    rospy.loginfo("Moving to place target")
    object_pose.pose.position.z += place_height
    self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, move_lin=True)
    object_pose.pose.position.z -= place_height

    # print "============ Stopping at the placement height. Press `Enter` to keep moving moving the robot ..."
    # raw_input()

    #gripper open
    if gripper_command=="complex_pick_from_inside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_close()
    elif gripper_command=="complex_pick_from_outside":
      self.precision_gripper_outer_open()
      self.precision_gripper_inner_open()
    elif gripper_command=="easy_pick_only_inner" or gripper_command=="inner_gripper_from_inside":
      self.precision_gripper_inner_close()
    elif gripper_command=="easy_pick_outside_only_inner" or gripper_command=="inner_gripper_from_outside":
      self.precision_gripper_inner_open()
    elif gripper_command=="none":
      pass
    else: 
      self.send_gripper_command(gripper=robotname, command="open")

    
    if lift_up_after_place:
      rospy.loginfo("Moving back up")
      object_pose.pose.position.z += approach_height
      self.go_to_pose_goal(robotname, object_pose, speed=speed_fast, move_lin=True)  
      object_pose.pose.position.z -= approach_height
    return True

  ######

  def do_pick_action(self, robot_name, pose_stamped, screw_size = 0, z_axis_rotation = 0.0, use_complex_planning = False, tool_name = ""):
    # Call the pick action
    goal = o2as_msgs.msg.pickGoal()
    goal.robot_name = robot_name
    goal.item_pose = pose_stamped
    goal.tool_name = tool_name
    goal.screw_size = screw_size
    goal.use_complex_planning = use_complex_planning
    goal.z_axis_rotation = z_axis_rotation
    rospy.loginfo("Sending pick action goal")
    rospy.logdebug(goal)

    self.pick_client.send_goal(goal)
    rospy.logdebug("Waiting for result")
    self.pick_client.wait_for_result()
    rospy.logdebug("Getting result")
    return self.pick_client.get_result()

  def do_place_action(self, robot_name, pose_stamped, tool_name = "", screw_size=0):
    # Call the pick action
    goal = o2as_msgs.msg.placeGoal()
    goal.robot_name = robot_name
    goal.item_pose = pose_stamped
    goal.tool_name = tool_name
    goal.screw_size = screw_size
    rospy.loginfo("Sending place action goal")
    rospy.logdebug(goal)

    self.place_client.send_goal(goal)
    rospy.logdebug("Waiting for result")
    self.place_client.wait_for_result()
    rospy.logdebug("Getting result")
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

  def do_change_tool_action(self, robot_name, equip=True, 
                        screw_size = 4):
    goal = o2as_msgs.msg.changeToolGoal()
    goal.robot_name = robot_name
    goal.equip_the_tool = equip
    goal.screw_size = screw_size
    rospy.loginfo("Sending changeTool action goal.")    
    self.change_tool_client.send_goal(goal)
    self.change_tool_client.wait_for_result()
    return self.change_tool_client.get_result()
  
  def do_screw_action(self, robot_name, target_hole, screw_height = 0.02, 
                        screw_size = 4):
    goal = o2as_msgs.msg.screwGoal()
    goal.target_hole = target_hole
    goal.screw_height = screw_height
    goal.screw_size = screw_size
    goal.robot_name = robot_name
    rospy.loginfo("Sending screw action goal.")
    self.screw_client.send_goal(goal)
    self.screw_client.wait_for_result()
    return self.screw_client.get_result()

  def set_motor(self, motor_name, direction = "tighten", wait=False, speed = 0, duration = 0):
    goal = o2as_msgs.msg.FastenerGripperControlGoal()
    goal.fastening_tool_name = motor_name
    goal.direction = direction
    goal.speed = speed
    goal.duration = duration
    rospy.loginfo("Sending fastening_tool action goal.")
    self.fastening_tool_client.send_goal(goal)
    if wait:
      self.fastening_tool_client.wait_for_result()
    return self.fastening_tool_client.get_result()

  def do_nut_fasten_action(self, item_name, wait = False):
    goal = o2as_msgs.msg.ToolsCommandGoal()
    # goal.stop = stop
    goal.peg_fasten = (item_name == "peg")
    # goal.big_nut_fasten = (item_name == "m10_nut")
    goal.big_nut_fasten = True
    goal.small_nut_fasten = (item_name == "m6_nut")
    rospy.loginfo("Sending nut_tool action goal.")
    self.nut_peg_tool_client.send_goal(goal)
    if wait:
      self.nut_peg_tool_client.wait_for_result(rospy.Duration.from_sec(30.0))
    return self.nut_peg_tool_client.get_result()

  def do_insertion(self, robot_name, max_insertion_distance=0.01, 
                        max_approach_distance = .1, max_force = 5,
                        max_radius = .001, radius_increment = .0001,
                        wait = False):
    # Directly calls the UR service rather than the action of the skill_server
    req = o2as_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.program_id = "insertion"
    # req.max_insertion_distance = max_insertion_distance
    # req.max_approach_distance = max_approach_distance
    # req.max_force = max_force
    # req.max_radius = max_radius
    # req.radius_increment = radius_increment
    res = self.urscript_client.call(req)
    if wait:
      wait_for_UR_program("/" + robot_name +"_controller", rospy.Duration.from_sec(30.0))
    return res.success
  
  def do_linear_push(self, robot_name, force, wait = False, direction = "Z+"):
    # Directly calls the UR service rather than the action of the skill_server
    req = o2as_msgs.srv.sendScriptToURRequest()
    req.robot_name = robot_name
    req.max_force = force
    req.force_direction = direction
    req.program_id = "linear_push"
    res = self.urscript_client.call(req)
    if wait:
      rospy.sleep(2.0)    # This program seems to take some time
      wait_for_UR_program("/" + robot_name +"_controller", rospy.Duration.from_sec(30.0))
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
  
  def send_gripper_command(self, gripper, command, this_action_grasps_an_object = False, force = 5.0, velocity = .1):
    if gripper == "precision_gripper_outer" or gripper == "precision_gripper_inner" or gripper == "a_bot":
      goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
      if command == "stop":
        goal.stop = True
      elif command == "close":
        if gripper == "precision_gripper_inner" or gripper == "a_bot":
          goal.close_inner_gripper_fully = True
        else:
          goal.close_outer_gripper_fully = True
      elif command == "open":
        if gripper == "precision_gripper_inner" or gripper == "a_bot":
          goal.open_inner_gripper_fully = True
        else:
          goal.open_outer_gripper_fully = True
      goal.this_action_grasps_an_object = this_action_grasps_an_object
      action_client = self.gripper_action_clients["a_bot"]
    elif gripper == "b_bot" or gripper == "c_bot":
      goal = robotiq_msgs.msg.CModelCommandGoal()
      action_client = self.gripper_action_clients[gripper]
      goal.velocity = velocity   # from 0.013 to 0.1
      goal.force = force
      if command == "close":
        goal.position = 0.0
      elif command == "open":
        goal.position = 0.085
      else:
        goal.position = command     # This sets the opening width directly
        rospy.loginfo(command)
    else:
      try:
        rospy.logerr("Could not parse gripper command: " + command + " for gripper " + gripper)
      except:
        pass

    action_client.send_goal(goal)
    rospy.loginfo("Sending command " + str(command) + " to gripper: " + gripper)
    action_client.wait_for_result(rospy.Duration(6.0))  # Default wait time: 6 s
    result = action_client.get_result()
    rospy.loginfo(result)
    return 

  def precision_gripper_outer_close(self):
    try:
        goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        goal.close_outer_gripper_fully = True
        goal.open_outer_gripper_fully = False
        self.gripper_action_clients["a_bot"].send_goal(goal)
        rospy.loginfo("close outer gripper")
        self.gripper_action_clients["a_bot"].wait_for_result(rospy.Duration(3.0))
        result = self.gripper_action_clients["a_bot"].get_result()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)


  def precision_gripper_outer_open(self):
    try:
        goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        goal.open_outer_gripper_fully = True
        goal.close_outer_gripper_fully = False
        self.gripper_action_clients["a_bot"].send_goal(goal)
        rospy.loginfo("open outer gripper")
        self.gripper_action_clients["a_bot"].wait_for_result(rospy.Duration(3.0))
        result = self.gripper_action_clients["a_bot"].get_result()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)

  def precision_gripper_inner_close(self, this_action_grasps_an_object = False):
    try:
        goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        goal.close_inner_gripper_fully = True
        goal.this_action_grasps_an_object = this_action_grasps_an_object
        self.gripper_action_clients["a_bot"].send_goal(goal)
        rospy.loginfo("Closing inner gripper")
        self.gripper_action_clients["a_bot"].wait_for_result(rospy.Duration(3.0))
        result = self.gripper_action_clients["a_bot"].get_result()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)


  def precision_gripper_inner_open(self, this_action_grasps_an_object = False):
    try:
        goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        goal.open_inner_gripper_fully = True
        goal.close_inner_gripper_fully = False
        goal.this_action_grasps_an_object = this_action_grasps_an_object
        self.gripper_action_clients["a_bot"].send_goal(goal)
        rospy.loginfo("Opening inner gripper")
        self.gripper_action_clients["a_bot"].wait_for_result(rospy.Duration(3.0))
        result = self.gripper_action_clients["a_bot"].get_result()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)
  
  def precision_gripper_inner_open_slightly(self, this_action_grasps_an_object = False, open_range = 30):
    try:
      action_client = self.gripper_action_clients["a_bot"]
      goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
      goal.open_inner_gripper_slightly = True
      goal.open_inner_gripper_fully = False
      goal.close_inner_gripper_fully = False
      goal.this_action_grasps_an_object = this_action_grasps_an_object
      goal.slight_opening_width = open_range
      self.gripper_action_clients["a_bot"].send_goal(goal)
      rospy.loginfo("Opening inner gripper slightly")
      self.gripper_action_clients["a_bot"].wait_for_result(rospy.Duration(3.0))
      result = self.gripper_action_clients["a_bot"].get_result()
      rospy.loginfo(result)
    except rospy.ServiceException, e:
        rospy.logerror("Service call failed: %s", e)

