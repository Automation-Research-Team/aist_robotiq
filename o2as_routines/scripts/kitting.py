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

from o2as_routines.base import O2ASBaseRoutines

CroppedArea = namedtuple("CroppedArea", ["min_row", "max_row", "min_col", "max_col"])

part_poses_demo = {
  "part_4":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.07),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.045),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_5":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_6":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_7":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_8":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.025),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.01),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_9":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_10":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_11":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.04),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.02),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_12":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.009),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.009),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
    
  },
  "part_13":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.024),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0)),
    "goal_position": geometry_msgs.msg.Point(0, 0, 0.012),
    "goal_orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_14":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_15":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_16":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_17":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  },
  "part_18":
  {
    "position": geometry_msgs.msg.Point(0, 0, 0.02),
    "orientation": geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
  }
}

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
    self.suction = rospy.ServiceProxy("o2as_usb_relay/set_power", SetPower)

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
    self.switch_suction(True)
    rospy.sleep(1)

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
    self.switch_suction(False)
    rospy.sleep(1)

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
    speed_slow = 0.05

    for set_num in range(1,4):
      item_list = rospy.get_param("/set_"+str(set_num))
      rospy.loginfo(item_list)
      rospy.loginfo("kitintg set_"+str(set_num) +" started!")

      # TODO: insert vision system
      for item in item_list:
        if self.gripper_id[item] == "suction":
          object_pose = geometry_msgs.msg.PoseStamped()
          object_pose.header.frame_id = self.bin_id[item]
          rospy.logdebug(object_pose.header.stamp)
          object_pose.pose.position = copy.deepcopy(part_poses_demo[item]["position"])
          object_pose.pose.orientation = copy.deepcopy(part_poses_demo[item]["orientation"])
          self.pick(robot_name, self.gripper_id[item], object_pose, speed_fast, speed_slow)

          place_pose = geometry_msgs.msg.PoseStamped()
          place_pose.header.frame_id = self.tray_id[item]
          place_pose.pose.position = copy.deepcopy(part_poses_demo[item]["goal_position"])
          place_pose.pose.orientation = copy.deepcopy(part_poses_demo[item]["goal_orientation"])
          self.place(robot_name, self.gripper_id[item], place_pose, speed_fast, speed_slow)

      print(" ____  _   _ ____  ____  _____ _   _ ____  ")
      print("/ ___|| | | / ___||  _ \| ____| \ | |  _ \ ")
      print("\___ \| | | \___ \| |_) |  _| |  \| | | | |")
      print(" ___) | |_| |___) |  __/| |___| |\  | |_| |")
      print("|____/ \___/|____/|_|   |_____|_| \_|____/ ")
      print("                                           ")
                                                                                
      raw_input()

  def kitting_task(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

    self.pick_and_place_demo()

    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")

    # TODO

  @staticmethod
  def _get_approach_pose(grasp_pose, approach_height):
    approach_pose = copy.deepcopy(grasp_pose)
    approach_pose.pose.position.z += approach_height
    approach_pose.pose.orientation.x = -0.5
    approach_pose.pose.orientation.y = 0.5
    approach_pose.pose.orientation.z = 0.5
    approach_pose.pose.orientation.w = 0.5

    return approach_pose

  def _get_depth_image(self):
    """Get depth image from phoxi via ROS.

    The return value must include not only depth, but also x and y coordinate
    in order to get the position of the pose in the camera frame
    (e.g., "o2as_easy_handeye_b_bot"). `img_z` in the returned values
    corresponds to the depth image.
    """
    img_x = None
    img_y = None
    img_z = None

    return img_x, img_y, img_z

  def _get_cropped_area(self, object_id):
    """Get cropped region based on object id.

    The area includes the specific bin for an object to be picked.
    The corresponding area for each object type is assumed to be obtained
    in some way (it can be manually or automatic) just before the trial.
    """
    # self.cropped_areas might be a dict
    return self.cropped_areas[object_id]

  def _crop_image(self, img, cropped_area):
    """Crop image.

    `img` is a ndarray of the image. `cropped_area` is an instance of
    CroppedArea structure (namedtuple) defined in this file.
    """
    return img[cropped_area.min_row:cropped_area.max_row,
               cropped_area.min_col:cropped_area.max_col]

  def _get_suction_pose(self, img_x, img_y, img_z, camera_frame):
    """Get a good suction pose based on depth image (`img_z`).
    """
    # row and col correspond to a pixel
    row, col = self._get_suction_point_with_matlab(img_z)

    # Position in the camera frame
    x, y, z = img_x[row, col], img_y[row, col], img_z[row, col]

    # Prepare pose object as returned value
    # pose.orientation will be overwritten outside this method
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = camera_frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = -0.5
    pose.pose.orientation.y = 0.5
    pose.pose.orientation.z = 0.5
    pose.pose.orientation.w = 0.5

    return pose

  def _tranform_reference_frame(self, pose, ref_new):
    # Not implemented
    return pose

  def pick_with_phoxi(self, robot_name, object_id, speed_fast, speed_slow,
                      approach_height=0.03):
    """Pick an object whose ID is given by `object_id` with the suction gripper.
    """
    self.go_to_mid_point(robot_name, speed=speed_fast)

    # self.publish_marker(object_pose, "aist_vision_result")
    if robot_name == "b_bot":
      self.groups[robot_name].set_end_effector_link(
        robot_name + '_dual_suction_gripper_pad_link')

    # Get a good suction pose for suction hand based on depth image
    img_x, img_y, img_z = self._get_depth_image()
    cropped_area = self._get_cropped_area(object_id)
    img_x = self._crop_image(img_x, cropped_area)
    img_y = self._crop_image(img_y, cropped_area)
    img_z = self._crop_image(img_z, cropped_area)
    suction_pose = self.get_suction_pose(img_x, img_y, img_z)

    # Transform reference frame for making control the orientation intuitive
    reference_new = robot_name + "base_link"
    suction_pose = self._transform_reference_frame(suction_pose, reference_new)

    # Set approach pose based on grasp pose
    approach_pose = self._set_approach_pose(suction_pose, approach_height)

    rospy.loginfo("Going above object to pick")
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)

    rospy.loginfo("Moving down to object")
    self.go_to_pose_goal(robot_name, suction_pose, speed=speed_slow,
                         high_precision=True)

    rospy.loginfo("Picking up on suction")
    # self.switch_suction(True)
    rospy.sleep(2)

    rospy.loginfo("Going back up")
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)


if __name__ == '__main__':
  try:
    kit = KittingClass()
    kit.set_up_item_parameters()
    
    kit.kitting_task()

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
