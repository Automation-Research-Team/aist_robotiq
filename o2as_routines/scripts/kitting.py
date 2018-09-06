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
    self.set_up_place_position()
    rospy.sleep(.5)

  def set_up_item_parameters(self):
    self.item_names = []
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 
    
  def set_up_end_effector(self):
    self.suction = rospy.ServiceProxy("o2as_usb_relay_server/set_power", SetPower)

  def set_up_place_position(self):
    self.part_id = [i for i in range(4,17,1)]
    self.place_id = [
      "set3_tray_1_partition_4",
      "set3_tray_2_partition_6",
      "set3_tray_1_partition_3",
      "set3_tray_1_partition_2",
      "set3_tray_2_partition_1",
      "set3_tray_2_partition_4",
      "set3_tray_2_partition_7",
      "set3_tray_1_partition_1",
      "set3_tray_2_partition_3",
      "set3_tray_1_partition_5",
      "set3_tray_2_partition_2",
      "set3_tray_2_partition_5",
      "set3_tray_2_partition_8"
    ]
    
  def go_to_mid_point(self, robot_name, speed=1.0):
    mid_pose = geometry_msgs.msg.PoseStamped()
    mid_pose.header.frame_id = "workspace_center"
    mid_pose.pose.orientation.x = -0.5
    mid_pose.pose.orientation.y = 0.5
    mid_pose.pose.orientation.z = 0.5
    mid_pose.pose.orientation.w = 0.5
    for i in np.arange(0.5, 0.09, -0.05):
      mid_pose.pose.position.z = i
      self.go_to_pose_goal(robot_name, mid_pose, speed=speed)

  ################ ----- Routines  
  ################ 
  ################ 

  def switch_suction(self, on=False):
    return self.suction(1, on)

  def pick(self, robot_name, object_id, object_pose, speed_fast, speed_slow, approach_height=0.03):

    self.go_to_mid_point(robot_name, speed=speed_fast)

    self.publish_marker(object_pose, "aist_vision_result")
    if robot_name=="b_bot":
      self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

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

  def place(self, robot_name, object_id, place_height, speed_fast, speed_slow, approach_height=0.05):

    if object_id < 3 and object_id > 16:
      rospy.logerr("This object_id is wrong!!")
      return
    self.go_to_mid_point(robot_name, speed=speed_fast)
    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = self.place_id[self.part_id.index(int(object_id))]
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

  def check_accessibility_by_b_bot(self):
    object_pose = geometry_msgs.msg.PoseStamped()

    object_pose.header.frame_id = "set1_bin3_1"
    object_pose.pose.position.x = -0.1
    object_pose.pose.position.y = -0.05
    object_pose.pose.position.z = 0.01
    object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, pi/2, 0))
    rospy.loginfo("Picking up an demo object.")
    self.pick("b_bot", 4, object_pose, 1.0, 1.0)
    tray_id = "set3_tray_1_partition_2"
    rospy.loginfo("Place down an demo object on tray partition.")
    self.place("b_bot", 4, 0.005, 1.0, 1.0)

    object_pose.header.frame_id = "set2_bin1_5"
    object_pose.pose.position.x = 0.03
    object_pose.pose.position.y = -0.04
    object_pose.pose.position.z = 0.01
    object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/4, pi/4, 0))
    rospy.loginfo("Picking up an demo object.")
    self.pick("b_bot", 16, object_pose, 1.0, 1.0)
    rospy.loginfo("Place down an demo object on tray partition.")
    self.place("b_bot", 16, 0.005, 1.0, 1.0)

    
    object_pose.header.frame_id = "set2_bin1_1"
    object_pose.pose.position.x = 0.03
    object_pose.pose.position.y = 0.04
    object_pose.pose.position.z = 0.01
    object_pose.pose.orientation.x = -0.5
    object_pose.pose.orientation.y = 0.5
    object_pose.pose.orientation.z = 0.5
    object_pose.pose.orientation.w = 0.5
    rospy.loginfo("Picking up an demo object.")
    self.pick("b_bot", object_pose, 1.0, 1.0)
    tray_id = "set3_tray_1_partition_2"
    rospy.loginfo("Place down an demo object on tray partition.")
    self.place("b_bot", tray_id, 0.005, 1.0, 1.0)

    object_pose.header.frame_id = "set1_bin2_1"
    object_pose.pose.position.x = -0.08
    object_pose.pose.position.y = 0.03
    object_pose.pose.position.z = 0.01
    rospy.loginfo("Picking up an demo object.")
    self.pick("b_bot", object_pose, 1.0, 1.0)
    tray_id = "set3_tray_1_partition_2"
    rospy.loginfo("Place down an demo object on tray partition.")
    self.place("b_bot", tray_id, 0.005, 1.0, 1.0)

  def pick_and_place_demo(self):

    robot_name = "b_bot"
    speed_fast = 1.0
    speed_slow = 1.0

    item_list = {
      "part_4": "set1_bin2_1",
      "part_8": "set1_bin2_2",
      "part_11": "set1_bin2_3",
      "part_13": "set1_bin2_4",
      "part_6": "set1_bin3_1",
      "part_9": "set2_bin1_1",
      "part_12": "set2_bin1_2",
      "part_16": "set2_bin1_3"
    }

    items = item_list.keys()
    part_ids = [i.strip("part_") for i in items]
    
    object_pose = geometry_msgs.msg.PoseStamped()
    object_pose.pose.position.z = 0.03
    object_pose.pose.orientation.x = -0.5
    object_pose.pose.orientation.y = 0.5
    object_pose.pose.orientation.z = 0.5
    object_pose.pose.orientation.w = 0.5

    for idx in part_ids:
      object_pose.header.frame_id = item_list["part_"+str(idx)]
      self.pick(robot_name, idx, object_pose, speed_fast, speed_slow)
      self.place(robot_name, idx, 0.03, speed_fast, speed_slow)

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
