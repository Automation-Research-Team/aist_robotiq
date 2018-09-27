#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi
import numpy as np
from collections import namedtuple

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg
from o2as_usb_relay.srv import *
from o2as_graspability_estimation.srv import *

from o2as_routines.base import O2ASBaseRoutines

LOG_LEVEL = log_level = rospy.DEBUG
# LOG_LEVEL = log_level = rospy.INFO

CroppedArea = namedtuple("CroppedArea", ["min_row", "max_row", "min_col", "max_col"])

# Temporary variable for integration of graspability estimation in AIST
bin_id_for_graspability_estimation = {
  "part_7": 1,
  "part_13": 2,
  "part_11": 3,
  "part_8": 4,
  "part_4": 5,
  "part_14": 6,
  "part_17": 7,
  "part_5": 8,
  "part_12": 9,
  "part_9": 10
}

part_poses_demo = {
  "part_4":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.07),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_5":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_6":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_7":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_8":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.024),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_9":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_10":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_11":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.037),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_12":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.009),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
    
  },
  "part_13":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.024),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_14":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_15":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_16":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_17":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_18":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.05),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  }
}

class KittingClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the kitting task. See base.py for shared convenience functions.
  """
  def __init__(self):

    super(KittingClass, self).__init__()
    
    # params
    self.bin_id = rospy.get_param('part_bin_list')
    self.gripper_id = rospy.get_param('gripper_id')
    self.tray_id = rospy.get_param('tray_id')
    
    # services
    self._suction = rospy.ServiceProxy("o2as_usb_relay/set_power", SetPower)
    self._search_grasp = rospy.ServiceProxy("search_grasp", SearchGrasp)

    self.set_up_item_parameters()
    rospy.sleep(.5)

  def set_up_item_parameters(self):
    self.item_names = []
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 

  ################ ----- Routines  
  ################ 
  ################ 

  def switch_suction(self, on=False):
    # Judge success or fail using pressure status.
    
    return self._suction(1, on)

  def pick_using_dual_suction_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link=""):
    rospy.loginfo("Go to the target.")
    res = self.move_lin(group_name, pose_goal_stamped, speed_slow, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to the target.")
      return False
    res = self.switch_suction(True)
    rospy.sleep(1)
    if not res:
      rospy.logdebug("Couldn't pick the target using suction.")
      return False
  
  def view_bin(self, group_name, bin_id, speed_fast = 1.0, speed_slow = 1.0, bin_eff_height = 0.3, bin_eff_xoff = 0, bin_eff_deg_angle = 20,end_effector_link = ""):
    # TODO: adjust the x,z and end effector orientatio for optimal view of the bin to use with the  \search_grasp service
    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = bin_id
    goal_pose.pose.position.x = bin_eff_xoff
    goal_pose.pose.position.y = 0
    goal_pose.pose.position.z = bin_eff_height
    #goal orientation for a_bot_camera_depth_frame 
    #goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2 + 20*pi/180, 0))
    #res = self.go_to_pose_goal(group_name, goal_pose, speed_slow, "a_bot_camera_depth_frame")
    #goal orientation for gripper
    goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2 + 20*pi/180, 0))
    res = self.move_lin(group_name, goal_pose, speed_slow, "")
    if not res:
      rospy.logdebug("Couldn't go to the target.")
      return False

  def pick_using_precision_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link = ""):
    # TODO: here is for Osaka Univ.
    pass

  def place_using_dual_suction_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link = ""):
    rospy.loginfo("Go to the target.")
    res = self.move_lin(group_name, pose_goal_stamped, speed, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to the target.")
      return False
    res = self.switch_suction(False)
    rospy.sleep(1)
    if not res:
      return False

  def place_using_precision_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link =""):
    # TODO: here is for Osaka Univ.
    pass

  def pick(self, group_name, pose_goal_stamped, speed_fast = 1.0, speed_slow = 1.0, end_effector_link = "", approach_height = 0.15):

    rospy.loginfo("Go to above the target bin.")
    pose_goal_above = copy.deepcopy(pose_goal_stamped)
    pose_goal_above.pose.position.z = approach_height
    res = self.move_lin(group_name, pose_goal_above, speed_fast, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to above the target bin.")
      return False

    if end_effector_link == "dual_suction_gripper_pad_link":
      self.pick_using_dual_suction_gripper(group_name, pose_goal_stamped, speed_slow, end_effector_link, approach_height)
    elif end_effector_link == "gripper":
      self.pick_using_precision_gripper(group_name, pose_goal_stamped, speed_slow, end_effector_link, approach_height)

    rospy.loginfo("Go to above the target bin.")
    res = self.move_lin(group_name, pose_goal_above, speed_slow, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to above the target bin.")
      return False

    return True
    
  def place(self, group_name, pose_goal_stamped, speed_fast = 1.0, speed_slow = 1.0, end_effector_link = "", approach_height = 0.15):
    rospy.loginfo("Go to above the target tray partition.")
    pose_goal_above = copy.deepcopy(pose_goal_stamped)
    pose_goal_above.pose.position.z = approach_height
    res = self.move_lin(group_name, pose_goal_above, speed_slow, end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to above the target bin.")
      return False



    rospy.loginfo("Go to above the target tray partition.")
    res = self.move_lin(group_name, pose_goal_above, speed_fast, end_effector_link)
    # if not res:
    #   rospy.logdebug("Couldn't go to above the target bin.")
    #   return False

    rospy.loginfo("Go back to home.")
    res = self.go_to_named_pose("home", group_name, speed_fast)
    # if not res:
    #   rospy.logdebug("Couldn't go back to home.")
    #   return False

    return True

  ################ ----- Demos  
  ################ 
  ################ 

  def pick_and_place_demo(self):

    speed_fast = 1.0
    speed_slow = 1.0

    for set_num in range(1,4):
      item_list = rospy.get_param("/set_"+str(set_num))
      rospy.loginfo(item_list)
      rospy.loginfo("kitintg set_"+str(set_num) +" started!")

      for item in item_list:
        req_search_grasp = SearchGraspRequest()
        req_search_grasp.part_id = int(str(item).strip("part_"))
        req_search_grasp.bin_id = int(bin_id_for_graspability_estimation[item])
        req_search_grasp.filename = str(rospy.Time.now()) + ".tiff"
        req_search_grasp.gripper = "suction"
        resp_search_grasp = self._search_grasp(req_search_grasp)

        if self.gripper_id[item] == "suction":
          # NOTE: should be given object_position from vision package (o2as_graspability_estimation)
          object_position = geometry_msgs.msg.PointStamped()
          object_position.header.frame_id = "a_phoxi_m_sensor"
          object_position.point = geometry_msgs.msg.Point(
            resp_search_grasp.pos3D[0].x, 
            resp_search_grasp.pos3D[0].y, 
            resp_search_grasp.pos3D[0].z)
          rospy.logdebug("\nGrasp point in %s: (x, y, z) = (%f, %f, %f)", 
            object_position.header.frame_id, 
            object_position.point.x, 
            object_position.point.y, 
            object_position.point.z)

          # Transform object_position to goal_pose in bin
          goal_pose = geometry_msgs.msg.PoseStamped()
          goal_pose.header.frame_id = "world"
          goal_pose.pose.position = self.listener.transformPoint(goal_pose.header.frame_id, object_position).point
          goal_pose.pose.orientation = copy.deepcopy(self.downward_orientation)
          rospy.logdebug("\nGrasp point in %s: (x, y, z) = (%f, %f, %f)", 
            goal_pose.header.frame_id, 
            goal_pose.pose.position.x, 
            goal_pose.pose.position.y, 
            goal_pose.pose.position.z)
          # FIXME: if finished check errors between scene and real, you must remove code about world_pose and robot_pose
          world_pose = geometry_msgs.msg.PoseStamped()
          world_pose.header.frame_id = "world"
          world_pose.pose.position = self.listener.transformPoint(world_pose.header.frame_id, object_position).point
          world_pose.pose.orientation = copy.deepcopy(self.downward_orientation)
          rospy.logdebug("\nGrasp point in %s: (x, y, z) = (%f, %f, %f)", 
            world_pose.header.frame_id, 
            world_pose.pose.position.x, 
            world_pose.pose.position.y, 
            world_pose.pose.position.z)
          
          res = self.pick("b_bot", goal_pose, speed_fast, speed_slow, "b_bot_dual_suction_gripper_pad_link", 0.1)
          robot_pose = self.groups["b_bot"].get_current_pose("b_bot_dual_suction_gripper_pad_link")
          rospy.logdebug("\nrobot ee position in %s: (x, y, z) = (%s, %s, %s)", 
            robot_pose.header.frame_id, 
            robot_pose.pose.position.x, 
            robot_pose.pose.position.y, 
            robot_pose.pose.position.z)
          if not res:
            rospy.logerr("Failed to pick target object.")
            return
          raw_input()

          self.go_to_named_pose("home", "b_bot")

          # place_pose = geometry_msgs.msg.PoseStamped()
          # place_pose.header.frame_id = self.tray_id[item]
          # place_pose.pose.position = copy.deepcopy(part_poses_demo[item]["goal_position"])
          # place_pose.pose.orientation = copy.deepcopy(part_poses_demo[item]["goal_orientation"])
          # res = self.place("b_bot", place_pose, speed_fast, speed_slow, "b_bot_dual_suction_gripper_pad_link")
          # if not res:
          #   rospy.logerr("Failed to place target object.")
          #   return
          # raw_input()

      print(" ____  _   _ ____  ____  _____ _   _ ____  ")
      print("/ ___|| | | / ___||  _ \| ____| \ | |  _ \ ")
      print("\___ \| | | \___ \| |_) |  _| |  \| | | | |")
      print(" ___) | |_| |___) |  __/| |___| |\  | |_| |")
      print("|____/ \___/|____/|_|   |_____|_| \_|____/ ")
      print("                                           ")

      raw_input()
  
  def kitting_task(self):
    # self.go_to_named_pose("home", "c_bot")
    # self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

    #self.pick_and_place_demo()
    self.view_bin("a_bot", "set2_bin1_2")

    # self.go_to_named_pose("home", "c_bot")
    # self.go_to_named_pose("home", "b_bot")
    # self.go_to_named_pose("home", "a_bot")

    # TODO


if __name__ == '__main__':

  try:
    kit = KittingClass()
    kit.set_up_item_parameters()
    
    kit.kitting_task()
    # kit.check_point_test()
    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
