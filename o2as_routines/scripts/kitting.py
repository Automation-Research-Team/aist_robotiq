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

  def pick(self, robot_name, gripper_name, object_pose, speed_fast, speed_slow, approach_height=0.03):
    
    if gripper_name=="suction":
      self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    self.publish_marker(object_pose, "aist_vision_result")
    rospy.loginfo("Going above object to pick")
    approach_pose = geometry_msgs.msg.PoseStamped()
    approach_pose = copy.deepcopy(object_pose)
    approach_pose.pose.position.z += approach_height
    approach_pose.pose.orientation.x = -0.5
    approach_pose.pose.orientation.y = 0.5
    approach_pose.pose.orientation.z = 0.5
    approach_pose.pose.orientation.w = 0.5
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)

    rospy.loginfo("Moving down to object")
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, high_precision=True)
    
    rospy.loginfo("Picking up on suction")
    # self.switch_suction(True)
    rospy.sleep(2)

    rospy.loginfo("Going back up")
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)

  def place(self, robot_name, gripper_name, place_id, place_height, speed_fast, speed_slow, approach_height=0.05):

    if gripper_name=="suction":
      self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = place_id
    goal_pose.pose.orientation.x = -0.5
    goal_pose.pose.orientation.y = 0.5
    goal_pose.pose.orientation.z = 0.5
    goal_pose.pose.orientation.w = 0.5
    goal_pose.pose.position.z = approach_height
    self.go_to_pose_goal(robot_name, goal_pose, speed=speed_fast)

    rospy.loginfo("Moving down to object")
    goal_pose.pose.position.z = place_height
    self.go_to_pose_goal(robot_name, goal_pose, speed=speed_slow, high_precision=True)
    rospy.loginfo("Place down from suction")
    # self.switch_suction(False)
    rospy.sleep(2)

    rospy.loginfo("Going back up")
    goal_pose.pose.position.z += approach_height
    self.go_to_pose_goal(robot_name, goal_pose, speed=speed_fast)

    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

  ################ ----- Demos  
  ################ 
  ################ 

  def pick_and_place_demo(self):

    robot_name = "b_bot"
    self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    speed_fast = 1.0
    speed_slow = 1.0

    bin_id = rospy.get_param('part_bin_list')
    gripper_id = rospy.get_param('gripper_id')
    tray_id = rospy.get_param('tray_id')

    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.pose.position.z = 0.01
    object_pose.pose.orientation.x = -0.5
    object_pose.pose.orientation.y = 0.5
    object_pose.pose.orientation.z = 0.5
    object_pose.pose.orientation.w = 0.5

    intermediate_pose = geometry_msgs.msg.PoseStamped()
    intermediate_pose.header.frame_id = "workspace_center"
    intermediate_pose.pose.position.z = 0.3
    intermediate_pose.pose.orientation.x = -0.5
    intermediate_pose.pose.orientation.y = 0.5
    intermediate_pose.pose.orientation.z = 0.5
    intermediate_pose.pose.orientation.w = 0.5

    for set_num in range(1,4):
      item_list = rospy.get_param("/set_"+str(set_num))
      rospy.loginfo("kitintg set_"+str(set_num) +" started!")

    # TODO: insert vision system
      for item in item_list:
        object_pose.header.frame_id = bin_id[item]
        if self.gripper_id[item] == "suction":
          self.go_to_pose_goal(robot_name, item, mediate_pose, speed_fast)
          self.pick(robot_name, gripper_id[item], item, object_pose, speed_fast, speed_slow)
          self.go_to_pose_goal(robot_name, intermediate_pose, speed_fast)
          self.place(robot_name, gripper_id[item], self.place_id[item], 0.01, speed_fast, speed_slow)

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
