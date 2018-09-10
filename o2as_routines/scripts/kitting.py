#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi
import numpy as np

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg
from o2as_usb_relay.srv import *

from o2as_routines.base import O2ASBaseRoutines

class KittingClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the kitting task. See base.py for shared convenience functions.
  """
  def __init__(self):
    super(KittingClass, self).__init__()
    self.bin_id = rospy.get_param('part_bin_list')
    self.gripper_id = rospy.get_param('gripper_id')
    self.tray_id = rospy.get_param('tray_id')
    self.set_up_item_parameters()
    self.set_up_end_effector()
    rospy.sleep(.5)

  def set_up_item_parameters(self):
    self.item_names = []
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 
    
  def set_up_end_effector(self):
    self.suction = rospy.ServiceProxy("o2as_usb_relay_server/set_power", SetPower)

  ################ ----- Routines  
  ################ 
  ################ 

  def switch_suction(self, on=False):
    return self.suction(1, on)

  def pick(self, robot_name, gripper_name, goal_pose, speed_fast, speed_slow, approach_height=0.03):
    
    if gripper_name=="suction":
      self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    self.publish_marker(goal_pose, "aist_vision_result")
    rospy.loginfo("Going above object to pick")
    self.go_to_check_point(robot_name, 'before_pick', speed_fast)

    approach_pose = geometry_msgs.msg.PoseStamped()
    approach_pose = copy.deepcopy(goal_pose)
    approach_pose.pose.position.z += approach_height
    approach_pose.pose.orientation.x = -0.5
    approach_pose.pose.orientation.y = 0.5
    approach_pose.pose.orientation.z = 0.5
    approach_pose.pose.orientation.w = 0.5
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)

    rospy.loginfo("Moving down to object")
    self.go_to_pose_goal(robot_name, goal_pose, speed=speed_slow, high_precision=True)
    
    rospy.loginfo("Picking up on suction")
    # self.switch_suction(True)
    rospy.sleep(2)

    rospy.loginfo("Going back up")
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)
    self.go_to_check_point(robot_name, 'after_pick', speed_fast)


  def place(self, robot_name, gripper_name, goal_pose, speed_fast, speed_slow, approach_height=0.05):

    if gripper_name=="suction":
      self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    self.go_to_check_point(robot_name, 'before_place', speed_fast)

    approach_pose = geometry_msgs.msg.PoseStamped()
    approach_pose = copy.deepcopy(goal_pose)
    approach_pose.pose.position.z += approach_height
    approach_pose.pose.orientation.x = -0.5
    approach_pose.pose.orientation.y = 0.5
    approach_pose.pose.orientation.z = 0.5
    approach_pose.pose.orientation.w = 0.5
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)

    rospy.loginfo("Moving down to object")
    self.go_to_pose_goal(robot_name, goal_pose, speed=speed_slow, high_precision=True)
    rospy.loginfo("Place down from suction")
    # self.switch_suction(False)
    rospy.sleep(2)

    rospy.loginfo("Going back up")
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)
    self.go_to_check_point(robot_name, 'after_place', speed_fast)

    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

  ################ ----- Supply methods
  ################ 
  ################ 
  def go_to_check_point(self, robot_name, task, speed):
    pose = geometry_msgs.msg.PoseStamped()
    pose.pose.position.z = 0.15
    pose.pose.orientation.x = -0.5
    pose.pose.orientation.y = 0.5
    pose.pose.orientation.z = 0.5
    pose.pose.orientation.w = 0.5

    if task == "before_pick":
      pose.header.frame_id = "workspace_center"
      self.go_to_pose_goal(robot_name, pose, speed)
      pose.header.frame_id = "rack_bins_center"
      self.go_to_pose_goal(robot_name, pose, speed)
    elif task == "after_pick":
      pose.header.frame_id = "rack_bins_center"
      self.go_to_pose_goal(robot_name, pose, speed)
      pose.header.frame_id = "workspace_center"
      self.go_to_pose_goal(robot_name, pose, speed)
    elif task == "before_place":
      pose.header.frame_id = "workspace_center"
      self.go_to_pose_goal(robot_name, pose, speed)
      pose.header.frame_id = "rack_trays_center"
      self.go_to_pose_goal(robot_name, pose, speed)
    elif task == "after_place":
      pose.header.frame_id = "rack_trays_center"
      self.go_to_pose_goal(robot_name, pose, speed)
      pose.header.frame_id = "workspace_center"
      self.go_to_pose_goal(robot_name, pose, speed)


  ################ ----- Demos  
  ################ 
  ################ 

  def pick_and_place_demo(self):

    robot_name = "b_bot"
    self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    speed_fast = 1.0
    speed_slow = 1.0    

    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.pose.position.z = 0.01
    object_pose.pose.orientation.x = -0.5
    object_pose.pose.orientation.y = 0.5
    object_pose.pose.orientation.z = 0.5
    object_pose.pose.orientation.w = 0.5

    for set_num in range(1,4):
      item_list = rospy.get_param("/set_"+str(set_num))
      rospy.loginfo(item_list)
      rospy.loginfo("kitintg set_"+str(set_num) +" started!")

    # TODO: insert vision system
      for item in item_list:
        object_pose.header.frame_id = self.bin_id[item]
        if self.gripper_id[item] == "suction":
          self.pick(robot_name, self.gripper_id[item], object_pose, speed_fast, speed_slow)
          place_pose = geometry_msgs.msg.PoseStamped()
          place_pose.header.frame_id = self.tray_id[item]
          place_pose.pose.position.z = 0.01
          place_pose.pose.orientation.x = -0.5
          place_pose.pose.orientation.y = 0.5
          place_pose.pose.orientation.z = 0.5
          place_pose.pose.orientation.w = 0.5
          self.place(robot_name, self.gripper_id[item], place_pose, speed_fast, speed_slow)

  def kitting_task(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

    self.pick_and_place_demo()

    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

    # TODO

if __name__ == '__main__':
  try:
    kit = KittingClass()
    kit.set_up_item_parameters()
    
    kit.kitting_task()

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
