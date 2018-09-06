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

CroppedArea = namedtuple("CroppedArea", ["min_row, max_row, min_col, max_col"])

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

  ################ ----- Routines  
  ################ 
  ################ 

  def switch_suction(self, on=False):
    return self.suction(1, on)

  def pick(self, robot_name, object_id, object_pose, speed_fast, speed_slow, approach_height=0.03):
    
    if robot_name=="b_bot":
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

  def place(self, robot_name, object_id, place_height, speed_fast, speed_slow, approach_height=0.05):

    if object_id < 3 and object_id > 16:
      rospy.logerr("This object_id is wrong!!")
      return
    if robot_name=="b_bot":
      self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

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
    self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    speed_fast = 1.0
    speed_slow = 1.0

    item_list = {
      "part_4": "set1_bin2_1",
      "part_8": "set1_bin2_2",
      "part_11": "set1_bin2_3",
      "part_13": "set1_bin2_4",
      "part_9": "set2_bin1_1",
      "part_12": "set2_bin1_2",
      "part_16": "set2_bin1_3"
    }

    items = item_list.keys()
    part_ids = [i.strip("part_") for i in items]
    
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

    for idx in part_ids:
      object_pose.header.frame_id = item_list["part_"+str(idx)]
      self.go_to_pose_goal(robot_name, intermediate_pose, speed_fast)
      self.pick(robot_name, idx, object_pose, speed_fast, speed_slow)
      self.go_to_pose_goal(robot_name, intermediate_pose, speed_fast)
      self.place(robot_name, idx, 0.01, speed_fast, speed_slow)

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
