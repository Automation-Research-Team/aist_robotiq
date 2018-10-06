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
from o2as_msgs.msg import *
# from o2as_usb_relay.srv import *
from o2as_graspability_estimation.srv import *
from std_msgs.msg import Bool

from o2as_routines.base import O2ASBaseRoutines

import rospkg
rp = rospkg.RosPack()
import csv
import os
import random


import math
from geometry_msgs.msg import Polygon, Point32
from PIL import Image, ImageDraw

LOG_LEVEL = log_level = rospy.DEBUG
# LOG_LEVEL = log_level = rospy.INFO

CroppedArea = namedtuple("CroppedArea", ["min_row", "max_row", "min_col", "max_col"])

class kitting_order_entry():
  """
  Object that tracks if its order was fulfilled, and the number of attempts spent on it.
  """
  def __init__(self, part_id, set_number, number_in_set, bin_name, target_frame, ee_to_use, item_name, dropoff_height):
    self.part_id = part_id   # The part id
    self.set_number = set_number
    self.number_in_set = number_in_set
    self.bin_name = bin_name
    self.target_frame = target_frame
    self.ee_to_use = ee_to_use
    self.item_name = item_name
    self.dropoff_height = dropoff_height

    self.attempts = 0
    self.fulfilled = False

    self.tf_listener = tf.TransformListener()

class KittingClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the kitting task. See base.py for shared convenience functions.
  """
  def __init__(self):
    super(KittingClass, self).__init__()
    
    # params
    # self.bin_id = rospy.get_param('part_bin_list')
    # self.gripper_id = rospy.get_param('gripper_id')
    # self.tray_id = rospy.get_param('tray_id')
    
    #subscribe to gripper position
    self._inner_gripper_pos_sub = rospy.Subscriber("o2as_precision_gripper/inner_gripper_motor_pos", std_msgs.msg.Int32, self.update_motorPosition)
    self._motorPos = -1

    # services
    # self._suction = rospy.ServiceProxy("o2as_usb_relay/set_power", SetPower)
    self._suction = actionlib.SimpleActionClient('o2as_fastening_tools/suction_control', SuctionControlAction)
    self._suctioned = False
    self._suction_state = rospy.Subscriber("suction_tool/screw_suctioned", Bool, self._suction_state_callback)
    # self._suction.wait_for_server()
    self._search_grasp = rospy.ServiceProxy("search_grasp", SearchGrasp)
    # action
    self.blob_detection_client = actionlib.SimpleActionClient('blob_detection_action', o2as_msgs.msg.blobDetectionAction)
    # self.blob_detection_client.wait_for_server()  

    self.initial_setup()
    rospy.sleep(.5)
    rospy.loginfo("Kitting class started up!")

  def update_motorPosition(self, msg):
    self._motorPos = msg.data

  def initial_setup(self):
    ### First, set up internal parameters
    ### Then, read order file and create the list to iterate through
    self.grasp_strategy = {
        "part_4" : "suction", 
        "part_5" : "suction", 
        "part_6" : "robotiq_gripper", 
        "part_7" : "suction",
        "part_8" : "suction", 
        "part_9" : "precision_gripper_from_inside", 
        "part_10": "precision_gripper_from_inside", 
        "part_11": "suction",  # Can be one of the grippers
        "part_12": "suction",  # Can be precision_gripper_from_outside in inclined bin
        "part_13": "suction",  # Should be precision_gripper_from_inside
        "part_14": "precision_gripper_from_inside", 
        "part_15": "precision_gripper_from_inside", 
        "part_16": "precision_gripper_from_inside", 
        "part_17": "precision_gripper_from_outside", 
        "part_18": "precision_gripper_from_outside"}
      
    # How high the end effector should hover over the tray when delivering the item
    self.dropoff_heights = {
        "part_4" : 0.03, 
        "part_5" : 0.02, 
        "part_6" : 0.01, 
        "part_7" : 0.04,
        "part_8" : 0.01, 
        "part_9" : 0.005, 
        "part_10": 0.005, 
        "part_11": 0.01,
        "part_12": 0.01,
        "part_13": 0.01,
        "part_14": 0.005, 
        "part_15": 0.005, 
        "part_16": 0.005, 
        "part_17": 0.005, 
        "part_18": 0.005}

    # TODO: Remove this hard-coded assignment
    self.bin_id_for_graspability_estimation = {
      "part_7": 1,
      "part_13": 2,
      "part_11": 3,
      "part_8": 4,
      "part_4": 5,
      "part_14": 10,
      "part_17": 7,
      "part_5": 8,
      "part_12": 9,
      "part_9": 6,
    }


    self.part_bin_list = {
      "part_11": "bin2_3", 
      "part_12": "bin1_2", 
      "part_13": "bin2_4",
      "part_16": "bin1_3", 
      "part_17": "bin1_4", 
      "part_18": "bin1_1", 
      "part_4": "bin2_1",
      "part_6": "bin3_1", 
      "part_8": "bin2_2", 
      "part_9": "bin1_5" }
<<<<<<< HEAD
    # self.part_bin_list = rospy.get_param("kitting_parts_bin_list")
=======
>>>>>>> 7fc3fa356665fc13fd11cbbb3d53de1c649437f9


    self.part_position_in_tray = {
      "part_4" : "tray_1_partition_4",
      "part_5" : "tray_2_partition_6",
      "part_6" : "tray_1_partition_3",
      "part_7" : "tray_1_partition_2",
      "part_8" : "tray_2_partition_1",
      "part_9" : "tray_2_partition_4",
      "part_10": "tray_2_partition_7",
      "part_11": "tray_1_partition_1",
      "part_12": "tray_2_partition_3",
      "part_13": "tray_1_partition_5",
      "part_14": "tray_2_partition_2",
      "part_15": "tray_2_partition_5",
      "part_16": "tray_2_partition_8",
      "part_17": "tray_2_this_is_a_screw_so_should_be_ignored",
      "part_18": "tray_2_this_is_a_screw_so_should_be_ignored" }

    # This can be copied from kitting_part_bin_list_auto.yaml
    self.bin_is_inclined = {
      "bin2_3": False,
      "bin1_2": False,
      "bin2_4": False,
      "bin1_3": False,
      "bin1_4": False,
      "bin1_1": False,
      "bin2_1": False,
      "bin3_1": False,
      "bin2_2": False,
      "bin1_5": False
    }

    #TODO: Count up delivered screws per set, so that the dropoff location changes

    self.suction_orientation_from_behind = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    self.suction_orientation_from_45 = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/4))
    self.suction_orientation_from_side = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))
    self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))

    ### Create the item list
    self.order_list_raw, self.ordered_items = self.read_order_file()
    rospy.loginfo("Received order list:")
    rospy.loginfo(self.order_list_raw)

    ### Sort by grasp strategy
    self.screw_items = []
    self.screws = dict()
    self.screws["m3"] = []
    self.screws["m4"] = []
    self.suction_items = []
    self.robotiq_gripper_items = []
    self.precision_gripper_items = []
    for order_item in self.ordered_items:
      if order_item.part_id in [17,18]:
        rospy.loginfo("Appended item nr." + str(order_item.number_in_set) + " from set " + str(order_item.set_number) + " (part ID:" + str(order_item.part_id) + ") to list of screw items")
        self.screw_items.append(order_item)
        if order_item.part_id == 17:
          self.screws["m4"].append(order_item)
        elif order_item.part_id == 18:
          self.screws["m3"].append(order_item)
      elif order_item.ee_to_use == "suction":
        rospy.loginfo("Appended item nr." + str(order_item.number_in_set) + " from set " + str(order_item.set_number) + " (part ID:" + str(order_item.part_id) + ") to list of suction items")
        self.suction_items.append(order_item)
      elif order_item.ee_to_use == "robotiq_gripper":
        rospy.loginfo("Appended item nr." + str(order_item.number_in_set) + " from set " + str(order_item.set_number) + " (part ID:" + str(order_item.part_id) + ") to list of robotiq_gripper items")
        self.robotiq_gripper_items.append(order_item)
      elif "precision_gripper" in order_item.ee_to_use:
        rospy.loginfo("Appended item nr." + str(order_item.number_in_set) + " from set " + str(order_item.set_number) + " (part ID:" + str(order_item.part_id) + ") to list of precision gripper items")
        self.precision_gripper_items.append(order_item)
    

  def read_order_file(self):
    """Read in the order file, return kitting_list and order_entry_list.
       kitting_list is a list of lists with only the part IDs that are ordered. 
       order_entry_list is a list of kitting_entry_item objects."""

    order_entry_list = []
    kitting_list = []
    kitting_list.append([])
    kitting_list.append([])
    kitting_list.append([])
    
    with open(os.path.join(rp.get_path("o2as_scene_description"), "config", "kitting_order_file.csv"), 'r') as f:
      reader = csv.reader(f)
      header = next(reader)
      # [0, 1, 2, 3, 4] = ["Set", "No.", "ID", "Name", "Note"]
      for data in reader:
        kitting_list[int(data[0])-1].append(int(data[2]))
        order_entry_list.append(kitting_order_entry(part_id=int(data[2]), set_number=int(data[0]), 
                              number_in_set=int(data[1]),
                              bin_name=self.part_bin_list["part_" + data[2]],
                              target_frame="set_" + data[0] + "_" + self.part_position_in_tray["part_" + data[2]], 
                              ee_to_use=self.grasp_strategy["part_" + data[2]],
                              item_name=data[4],
                              dropoff_height=self.dropoff_heights["part_" + data[2]]) )
    return kitting_list, order_entry_list

    

  ################ ----- Routines  
  ################ 
  ################

  def pick(self, robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, special_pick = False):
    # If the pick uses suction, pass it to the local function. Otherwise to the parent class.
    if gripper_command == "suction":
      object_pose.pose.position.z += approach_height
      self.move_lin(robot_name, object_pose, speed_fast, end_effector_link="b_bot_suction_tool_tip_link")
      object_pose.pose.position.z -= approach_height
      picked = self.pick_using_dual_suction_gripper(robot_name, object_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
      object_pose.pose.position.z += approach_height
      self.move_lin(robot_name, object_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
      object_pose.pose.position.z -= approach_height
      bool_msg = Bool()
      try:
        bool_msg = rospy.wait_for_message("/screw_tool_m" + str(screw_size) + "/screw_suctioned", Bool, 1.0)
      except:
        pass
      screw_picked = bool_msg.data
      return picked
    else:
      return super(KittingClass, self).pick(robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height, special_pick)

  def place(self, robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, special_pick = False):
    # If the place uses suction, pass it to the local function. Otherwise to the parent class.
    if gripper_command == "suction":
      object_pose.pose.position.z += approach_height
      self.move_lin(robot_name, object_pose, speed_fast, end_effector_link="b_bot_suction_tool_tip_link")
      object_pose.pose.position.z -= approach_height
      picked = self.place_using_dual_suction_gripper(robot_name, object_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
      object_pose.pose.position.z += approach_height
      self.move_lin(robot_name, object_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
      object_pose.pose.position.z -= approach_height
      # TODO: 
      return picked
    else:
      return super(KittingClass, self).pick(robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height, special_pick)
    
  def _suction_state_callback(self, data):
    self._suctioned = data.data

  def suck(self, turn_suction_on=False):
    # Judge success or fail using pressure status.
    if not self.use_real_robot:
      return True
    
    goal = SuctionControlGoal()
    goal.fastening_tool_name = "suction_tool"
    goal.turn_suction_on = turn_suction_on
    goal.eject_screw = False
    self._suction.send_goal(goal)
    self._suction.wait_for_result()
    return self._suction.get_result()

  def pick_using_dual_suction_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link="b_bot_suction_tool_tip_link"):
    rospy.loginfo("Try picking up by suction.")
    res = self.suck(True)
    if not res:
      return False
    
    res = self.move_lin(group_name, pose_goal_stamped, speed, end_effector_link=end_effector_link)
    if not res:
      return False
  
  def place_using_dual_suction_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link = "b_bot_suction_tool_tip_link"):
    rospy.loginfo("Go to the target.")
    res = self.move_lin(group_name, pose_goal_stamped, speed, end_effector_link=end_effector_link)
    if not res:
      rospy.logdebug("Couldn't go to the target.")
      return False
    res = self.switch_suction(False)
    rospy.sleep(1)
    if not res:
      return False

    start_time = rospy.get_rostime()
    while ((rospy.get_rostime().secs - start_time.secs) <= 10.0):
      if self._suctioned:
        return True
    return False

  def naive_pick(self, group_name, bin_id, speed_fast = 1.0, speed_slow = 1.0, approach_height = 0.05, bin_eff_height = 0.07, bin_eff_xoff = 0, bin_eff_yoff = 0, bin_eff_deg_angle = 0,end_effector_link = ""):

    #Place gripper above bin
    goal_pose_above = geometry_msgs.msg.PoseStamped()
    goal_pose_above.header.frame_id = bin_id
    goal_pose_above.pose.position.x = bin_eff_xoff
    goal_pose_above.pose.position.y = bin_eff_yoff
    goal_pose_above.pose.position.z = bin_eff_height
    goal_pose_above.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2 , 0))
    res = self.move_lin(group_name, goal_pose_above, speed=speed_fast, end_effector_link=end_effector_link)
    if not res:
      rospy.loginfo("Couldn't go to the target.")
      return False

    #Open the gripper 
    self.send_gripper_command(gripper= "precision_gripper_inner", command="close")
    self.precision_gripper_inner_open()
    rospy.sleep(1)

    #Descend
    rospy.loginfo("Descend on target.")
    goal_pose_descend = copy.deepcopy(goal_pose_above)
    goal_pose_descend.pose.position.z = goal_pose_descend.pose.position.z - approach_height
    res  = self.move_lin(group_name, goal_pose_descend, speed=speed_slow, end_effector_link=end_effector_link)
    if not res:
      rospy.loginfo("Couldn't go to above the target bin.")
      return False

    #Close the gripper 
    self.send_gripper_command(gripper= "precision_gripper_inner", command="open")
    self.precision_gripper_inner_close()
    rospy.sleep(1)

    #Ascend
    rospy.loginfo("Ascend with target.")
    goal_pose_ascend = copy.deepcopy(goal_pose_descend)
    goal_pose_ascend.pose.position.z = goal_pose_ascend.pose.position.z + approach_height
    res  = self.move_lin(group_name, goal_pose_ascend, speed=speed_slow, end_effector_link=end_effector_link)
    if not res:
      rospy.loginfo("Couldn't go to above the target bin.")
      return False

  def pick_screw_precision_gripper(self, bin_id, screw_size, speed_fast = 1.0, speed_slow = .1, approach_height = 0.05, bin_eff_height = 0.07, bin_eff_deg_angle = 0,end_effector_link = ""):
    success_pick = False
    #Close the gripper and save the motor position
    self.send_gripper_command(gripper= "precision_gripper_inner", command="open")
    rospy.sleep(0.1)
    close_pos = self._motorPos

    posx = 0
    posy = 0
    posz = 0.08

    while (not success_pick):
      #Random pick
      self.naive_pick("a_bot", bin_id, speed_fast, speed_slow, approach_height, posz, posx, posy, bin_eff_deg_angle, end_effector_link)


      #Go to the bin center to start the screw checking
      rospy.loginfo("Go to the bin center to start the screw checking")
      goal_pose_incline = geometry_msgs.msg.PoseStamped()
      goal_pose_incline.header.frame_id = bin_id
      goal_pose_incline.pose.position.x = 0
      goal_pose_incline.pose.position.y = 0.035
      goal_pose_incline.pose.position.z = 0.30
      goal_pose_incline.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
      res = self.move_lin("a_bot", goal_pose_incline, speed_slow)

      #Check posture of the screw
      rospy.loginfo("Begin check motion")
      over_bin_3_joint_pose = [0.5639523868982476, -1.2023834734104668, 2.084380077110544, -4.115980903386012, -1.350262946004677, 1.5910085738144437]
      self.groups["a_bot"].set_joint_value_target(over_bin_3_joint_pose)
      self.groups["a_bot"].set_max_velocity_scaling_factor(.1)
      self.groups["a_bot"].go(wait=True)

      over_bin_3_inclined_joint_pose = [0.5646427393623301, -1.202333924982511, 2.040723585395901, -4.197616886811121, -1.3550737620026068, 1.6180429123653095]
      self.groups["a_bot"].set_joint_value_target(over_bin_3_inclined_joint_pose)
      self.groups["a_bot"].set_max_velocity_scaling_factor(.1)
      self.groups["a_bot"].go(wait=True)

      rospy.loginfo("Ending check motion")
      if not res:
        rospy.loginfo("Couldn't go to the target.")
        return False

      #Open just a bit the gripper
      if(screw_size == 4):
        open_range = 80
      elif(screw_size == 3):
        open_range = 60
      else:
        rospy.logerror("Screw size is wrong")
        open_range = 0
      rospy.loginfo("begin inner open slightly")
      self.precision_gripper_inner_open_slightly(open_range)
      rospy.loginfo("ending inner open slightly")
      rospy.sleep(1.0)
      #Close the gripper fully
      self.send_gripper_command(gripper= "precision_gripper_inner", command="close")
      rospy.sleep(1.0)
      #Check the motor position 
      success_pick = True
      if(abs(self._motorPos - close_pos) > 25 ):
        success_pick = True
      if not self.use_real_robot:
        rospy.loginfo("Assume success for the simulation")
        success_pick = True

    # goal_pose_pick = geometry_msgs.msg.PoseStamped()
    # goal_pose_pick.header.frame_id = "rack_trays_center"
    # goal_pose_pick.pose.position.x = 0
    # goal_pose_pick.pose.position.y = 0
    # goal_pose_pick.pose.position.z = 0.30
    # goal_pose_pick.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0.75*pi))
    # res = self.move_lin("a_bot", goal_pose_pick, speed_slow)
    # if not res:
    #   rospy.loginfo("Couldn't go to the pick position for the screw tool.")
    #   return False
    
    above_handover_joint_pose = [1.3983707427978516, -1.343994442616598, 2.048433780670166, -4.056087795888082, -1.9765618483172815, 1.4695862531661987]
    self.groups["a_bot"].set_joint_value_target(above_handover_joint_pose)
    self.groups["a_bot"].set_max_velocity_scaling_factor(.3)
    self.groups["a_bot"].go(wait=True)
    self.go_to_named_pose("screw_handover", "a_bot")
    self.precision_gripper_inner_open_slightly(open_range)
    return True
    
  def pick_screw_from_precision_gripper(self, screw_size, attempts = 1):
    """
    Picks a screw from the precision gripper.
    """
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4!")
      return False
    
    # Turn to the right to face the feeders
    self.go_to_named_pose("screw_ready", "c_bot")
    self.go_to_named_pose("screw_handover", "a_bot")
    # ATTENTION: MAGIC NUMBERS
    magic_y_offset = .004
    magic_z_offset = -.008

    pick_pose = geometry_msgs.msg.PoseStamped()
    pick_pose.header.frame_id = "a_bot_gripper_screw_pickup"
    pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/6, 0, 0))
    pick_pose.pose.position.y = magic_y_offset
    pick_pose.pose.position.z = magic_z_offset

    prep_pose = copy.deepcopy(pick_pose)
    prep_pose.pose.position.x = -0.03
    prep_pose.pose.position.y = -0.06
    
    print("press enter")
    raw_input()
    self.move_lin("c_bot", prep_pose, .2, end_effector_link="c_bot_screw_tool_m"+str(screw_size)+"_tip_link")
    prep_pose.pose.position.y = magic_y_offset
    prep_pose.pose.position.z = magic_z_offset
    self.move_lin("c_bot", prep_pose, .2, end_effector_link="c_bot_screw_tool_m"+str(screw_size)+"_tip_link")

    attempt = 0
    screw_picked = False
    while attempt < attempts:
      print("press enter")
      raw_input()
      self.do_pick_action("c_bot", pick_pose, screw_size = 4, use_complex_planning = True, tool_name = "screw_tool")
      bool_msg = Bool()
      try:
        bool_msg = rospy.wait_for_message("/screw_tool_m" + str(screw_size) + "/screw_suctioned", Bool, 1.0)
      except:
        pass
      screw_picked = bool_msg.data
      if screw_picked:
        rospy.loginfo("Successfully picked the screw")
        return True
      if not self.use_real_robot:
        rospy.loginfo("Pretending the screw is picked, because this is simulation.")
        return True
      attempt += 1

    return False

  def view_bin(self, group_name, bin_id, speed_fast = 1.0, speed_slow = 1.0, bin_eff_height = 0.2, bin_eff_xoff = 0, bin_eff_deg_angle = 20,end_effector_link = ""):
    # TODO: adjust the x,z and end effector orientatio for optimal view of the bin to use with the  \search_grasp service
    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = bin_id
    goal_pose.pose.position.x = bin_eff_xoff - .1
    goal_pose.pose.position.y = -.05
    goal_pose.pose.position.z = bin_eff_height - .12

    #goal orientation for a_bot_camera_depth_frame 
    #goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2 + 20*pi/180, 0))
    #res = self.go_to_pose_goal(group_name, goal_pose, speed_slow, "a_bot_camera_depth_frame")
    #goal orientation for gripper
#    goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2 + 20*pi/180, 0))
    goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2, 0))
    res = self.move_lin(group_name, goal_pose, speed_slow, end_effector_link="")
    if not res:
      rospy.loginfo("Couldn't go to the target.")
    #TODO Problem with gazebo controller while controlling the robot with movelin
      return False
 
    #TODO Ensure that the the motion is finished before generating the mask.

    #TODO not sure if the sleep is necessary is move_lin wait for the motion to be finished?
    rospy.sleep(2)

    mask_margin = 0.0
    point_top1 = geometry_msgs.msg.PointStamped()
    point_top1.header.frame_id = str(bin_id)+"_bottom_front_right_corner"
    point_top1.point = geometry_msgs.msg.Point(-mask_margin, -mask_margin, 0.0)

    point_top2 = geometry_msgs.msg.PointStamped()
    point_top2.header.frame_id = str(bin_id)+"_bottom_back_right_corner"
    point_top2.point = geometry_msgs.msg.Point(mask_margin, -mask_margin, 0.0)

    point_top3 = geometry_msgs.msg.PointStamped()
    point_top3.header.frame_id = str(bin_id)+"_bottom_back_left_corner"
    point_top3.point = geometry_msgs.msg.Point(mask_margin, mask_margin, 0.0)

    point_top4 = geometry_msgs.msg.PointStamped()
    point_top4.header.frame_id = str(bin_id)+"_bottom_front_left_corner"
    point_top4.point = geometry_msgs.msg.Point(-mask_margin, mask_margin, 0.0)

    #TODO change the fisheye from to the depth frame in casse of offset. but fisheye should e ok since the two images are aligned (depth and rgb) after the real sense node
    point_top1_cam = self.listener.transformPoint("a_bot_camera_fisheye_optical_frame", point_top1).point
    point_top2_cam = self.listener.transformPoint("a_bot_camera_fisheye_optical_frame", point_top2).point
    point_top3_cam = self.listener.transformPoint("a_bot_camera_fisheye_optical_frame", point_top3).point
    point_top4_cam = self.listener.transformPoint("a_bot_camera_fisheye_optical_frame", point_top4).point


    #print("point_top1_cam")
    #print(point_top1_cam)
    #print("point_top2_cam")
    #print(point_top2_cam)
    #print("point_top3_cam")
    #print(point_top3_cam)
    #print("point_top4_cam")
    #print(point_top4_cam)

    #point_test_center_cam = geometry_msgs.msg.Point(0, 0, 0.4)
    #print(point_test_center_cam)

    #TODO project the 4 points in the depth plane (which projection matrix to use from the camera)
    #projection matrix
    #554.3827128226441, 0.0, 320.5, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 1.0    
    #
    #P: [554.3827128226441, 0.0, 320.5, -38.80678989758509, 0.0, 554.3827128226441, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    #TODO take the parameters from the /camera_info topic instead


#for gazebo
#    cameraMatK = np.array([[554.3827128226441, 0.0, 320.5],
#                           [0.0, 554.3827128226441, 240.5],
#                           [0.0, 0.0, 1.0]])

#for ID Realsense on robot ID61*41   width 640 height 360
    cameraMatK = np.array([[461.605774, 0.0, 318.471497],
                           [0.0, 461.605804, 180.336258],
                           [0.0, 0.0, 1.0]])


    point_top1_cam_np = np.array([point_top1_cam.x, point_top1_cam.y, point_top1_cam.z])   
    point_top2_cam_np = np.array([point_top2_cam.x, point_top2_cam.y, point_top2_cam.z])   
    point_top3_cam_np = np.array([point_top3_cam.x, point_top3_cam.y, point_top3_cam.z])   
    point_top4_cam_np = np.array([point_top4_cam.x, point_top4_cam.y, point_top4_cam.z])   
    #used to test the projection
    #point_test_center_cam_np = np.array([point_test_center_cam.x, point_test_center_cam.y, point_test_center_cam.z])   
      
    point_top1_img_np = cameraMatK.dot(point_top1_cam_np)
    point_top2_img_np = cameraMatK.dot(point_top2_cam_np)
    point_top3_img_np = cameraMatK.dot(point_top3_cam_np)
    point_top4_img_np = cameraMatK.dot(point_top4_cam_np)
    #point_test_center_img_np = cameraMatK.dot(point_test_center_cam_np) 

    #print(point_top1_img_np)
    #print(point_top2_img_np)
    #print(point_top3_img_np)
    #print(point_top4_img_np)
    #print(point_test_center_img_np)

    mask_polygon = Polygon()
    mask_polygon.points = [Point32(point_top1_img_np[0]/point_top1_img_np[2],point_top1_img_np[1]/point_top1_img_np[2],0),
                    Point32(point_top2_img_np[0]/point_top2_img_np[2],point_top2_img_np[1]/point_top2_img_np[2],0),
                    Point32(point_top3_img_np[0]/point_top3_img_np[2],point_top3_img_np[1]/point_top3_img_np[2],0),
                    Point32(point_top4_img_np[0]/point_top4_img_np[2],point_top4_img_np[1]/point_top4_img_np[2],0)]

    goal = o2as_msgs.msg.blobDetectionGoal()
    goal.maskCorner = mask_polygon
    self.blob_detection_client.send_goal(goal)
    self.blob_detection_client.wait_for_result()
    result = self.blob_detection_client.get_result()
    rospy.loginfo(result)

    #TODO select which poses to choose in the array
    poseArrayRes = geometry_msgs.msg.PoseArray()    
    poseArrayRes = result.posesDetected 

    #TODO Sort pose in the midle of the bin
    #min x min y in the bin frame

    if(result.success): 

        distanceToBinCenter = []
        for i in range(len(poseArrayRes.poses)): 
            pointCam = geometry_msgs.msg.PointStamped()

            #simulation only
            poseArrayRes.header.frame_id = "a_bot_camera_fisheye_optical_frame"
            pointCam.header = poseArrayRes.header
            print("pointCam.header")
            print(pointCam.header)
            pointCam.point = poseArrayRes.poses[i].position
            pointBin = self.listener.transformPoint(bin_id, pointCam).point
            distanceToBinCenter.append(math.sqrt(pointBin.x*pointBin.x + pointBin.y*pointBin.y))
        minPoseIndex = np.argmin(distanceToBinCenter)
         
        rospy.loginfo("pose closest to the bin center in the xy plane")
        rospy.loginfo(poseArrayRes.poses[minPoseIndex])

        #Place gripper above bin
        pointPartCam = geometry_msgs.msg.PointStamped()
        pointPartCam.header = poseArrayRes.header
        pointPartCam.point = poseArrayRes.poses[i].position
        pointPartBin = self.listener.transformPoint(bin_id, pointPartCam)

        rospy.loginfo("Pose in bin")
        rospy.loginfo(pointPartBin)

        goal_part = geometry_msgs.msg.PoseStamped()
        goal_part.header.frame_id = pointPartBin.header.frame_id
        goal_part.pose.position.x = pointPartBin.point.x
        goal_part.pose.position.y = pointPartBin.point.y
        goal_part.pose.position.z = 0.01
        goal_part.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(-pi/2, pi/2 , 0))
        res = self.move_lin(group_name, goal_part, speed_slow, "")
        if not res:
          rospy.loginfo("Couldn't go to the target.")
        #TODO Problem with gazebo controller while controlling the robot with movelin
          return False

    else:
        rospy.loginfo("no pose detected")
    #TODO if nothing is detected move the camera a bit to try to detect somethin



    #mask_img = Image.new('L', (640,480), 0)
    #ImageDraw.Draw(mask_img).polygon(mask_polygon, outline = 1, fill = 255)
    #mask_image_np = np.array(mask_img)
    #namefile = "mask_"+str(bin_id)+".png"
    #mask_img.save(namefile,'PNG')


    #used to do the bitwise comparison between the mask and the original image
    #mask = Image.new('RGB', (640,480), 255)
    #ImageDraw.Draw(mask).polygon(polygon, outline = 1, fill = 1)
    #mask_np = np.array(mask)
 
    #img_original = Image.open("bin_origin.png")
    #img_original_np = np.array(img_original)
 
    #img_res_np = np.bitwise_and(mask_np, img_original_np)
    #img_res = Image.fromarray(np.uint8(img_res_np)) 
    #img_res.save('mask_original_bin.png','PNG')


    #    point_top1_cam = t.transformPoint("a_bot_camera_depth_frame", point_top1)
    #    point_top1_cam = t.transformPoint(point_top4.header.frame_id, point_top1)

  def pick_screw_from_feeder(self, screw_size, attempts = 1):
    """
    Picks a screw from one of the feeders. The screw tool already has to be equipped!
    """
    # Use this command to equip the screw tool: do_change_tool_action(self, "c_bot", equip=True, screw_size = 4)
    
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4!")
      return False
    
    # Turn to the right to face the feeders
    self.go_to_named_pose("feeder_pick_ready", "c_bot")

    pose_feeder = geometry_msgs.msg.PoseStamped()
    pose_feeder.header.frame_id = "m" + str(screw_size) + "_feeder_outlet_link"
    pose_feeder.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pose_feeder.pose.position.x = 0.0

    attempt = 0
    screw_picked = False
    while attempt < attempts:
      self.do_pick_action("c_bot", pose_feeder, screw_size = 4, use_complex_planning = True, tool_name = "screw_tool")
      bool_msg = Bool()
      try:
        bool_msg = rospy.wait_for_message("/screw_tool_m" + str(screw_size) + "/screw_suctioned", Bool, 1.0)
      except:
        pass
      screw_picked = bool_msg.data
      if screw_picked:
        rospy.loginfo("Successfully picked the screw")
        return True
      if not self.use_real_robot:
        rospy.loginfo("Pretending the screw is picked, because this is simulation.")
        return True
      attempt += 1

    return False

  def place_screw_in_tray(self, screw_size, set_number, hole_number):
    """
    Places a screw in a tray. A screw needs to be carried by the screw tool!
    """
    # Use this command to equip the screw tool: do_change_tool_action(self, "c_bot", equip=True, screw_size = 4)
    
    if not screw_size==3 and not screw_size==4:
      rospy.logerr("Screw size needs to be 3 or 4!")
      return False
    
    # Turn to the right to face the feeders
    self.go_to_named_pose("feeder_pick_ready", "c_bot")
    
    pose_tray = geometry_msgs.msg.PoseStamped()
    pose_tray.header.frame_id = "set_" + str(set_number) + "_tray_2_screw_m" + str(screw_size) + "_" + str(hole_number)
    if set_number == 1:
      pose_tray.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/2, 0, 0))
    elif set_number == 2:
      pose_tray.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, 0, 0))
    elif set_number == 3:
      pose_tray.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    pose_tray.pose.position.x = -.01

    success = self.do_place_action("c_bot", pose_tray, tool_name = "screw_tool", screw_size=screw_size)
    if not success:
      return False
    
    self.go_to_named_pose("feeder_pick_ready", "c_bot")
    return True
    

<<<<<<< HEAD
  def place_using_dual_suction_gripper(self, group_name, pose_goal_stamped, speed, end_effector_link = "b_bot_suction_tool_tip_link"):
    rospy.loginfo("Try place by suction.")
    res = self.suck(False)
    if not res:
      return False
    
    res = self.move_lin(group_name, pose_goal_stamped, speed, end_effector_link=end_effector_link)
    if not res:
      return False

    start_time = rospy.get_rostime()
    while ((rospy.get_rostime().secs - start_time.secs) <= 10.0):
      if not self._suctioned:
        return True
    return False


=======
>>>>>>> 7fc3fa356665fc13fd11cbbb3d53de1c649437f9
  def set_orientation_for_suction_tool(self, target_pose):
    """Takes a pose and assigns a suitable orientation for the suction"""
    # This needs to transform the pose to the world and check if it is too close to b_bot.
    pass

<<<<<<< HEAD
=======
  ################ ----- Demos  
  ################ 
  ################ 
>>>>>>> 7fc3fa356665fc13fd11cbbb3d53de1c649437f9

  def pick_screw_from_bin_and_put_into_feeder(self, item, max_attempts = 10):
    robot_name = "a_bot"
    end_effector_link = "a_bot_precision_gripper_tip_link"
    if item.part_id == 17:
      screw_size = 4
    elif item.part_id == 18:
      screw_size = 3
    attempts = 0
    while attempts < max_attempts:
      attempts += 1
      item.attempts += 1
      rospy.loginfo("Attempting to pick screw. Item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Attempt nr. " + str(item.attempts))
      
      # Attempt to pick the item
      pick_pose = geometry_msgs.msg.PoseStamped()
      pick_pose.header.frame_id = item.bin_name
      pick_pose.pose.orientation = self.downward_orientation
      gripper_command = "inner_gripper_from_outside"

      rospy.logerr("TODO: Turn off the feeder")

      self.pick(robot_name, pick_pose, 0.0, speed_fast = 0.3, speed_slow = 0.02, 
                        gripper_command=gripper_command, approach_height = 0.08)

      rospy.logerr("TODO: Turn on the feeder")
      bool_msg = Bool()
      try:
        bool_msg = rospy.wait_for_message("/screw_tool_m" + str(screw_size) + "/screw_suctioned", Bool, 1.0)
      except:
        pass
      screw_picked = bool_msg.data
      if not screw_picked or not self.use_real_robot:
        rospy.logerr("Failed an attempt to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Reattempting. Current attempts: " + str(attempts))
        continue
      
      # Put the screw in the feeder
      self.go_to_named_pose("home", "a_bot")
      drop_pose = geometry_msgs.msg.PoseStamped()
      drop_pose.pose.orientation = self.downward_orientation
      drop_pose.header.frame_id = "m" + str(screw_size) + "_feeder_inlet_link"

      self.place(robot_name, drop_pose, 0.04,
                    speed_fast = 0.3, speed_slow = 0.02, gripper_command=gripper_command, approach_height=0.0)
      self.fulfilled_items += 1
      item.fulfilled = True
      rospy.loginfo("Delivered item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")! Total items delivered: " + str(self.fulfilled_items))
      return True
    
    return False
  
  def get_grasp_position_from_phoxi(self, item, take_new_image=True):
    update_image = True  
    if item.ee_to_use == "suction":
      req_search_grasp = SearchGraspRequest()
      req_search_grasp.part_id = int(str(item.part_id).strip("part_"))
      rospy.logerr("Using hard-coded bin ID assignment for graspability search!")
      req_search_grasp.bin_id = int(bin_id_for_graspability_estimation[item.part_id])
      if item.ee_to_use == "suction":
        req_search_grasp.gripper = "suction"
      else:
        req_search_grasp.gripper = "two_finger"
        #TODO: Is this still correct? What about inside/outside grasping with the precision gripper?
      req_search_grasp.update_image = update_image
      try:
        resp_search_grasp = self._search_grasp(req_search_grasp)
        update_image = False
      except rospy.ServiceException as e:
        rospy.logerr(e.message)
        return False
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
    return object_position

  def get_item_pose(self, item, is_update=True):
    req = SearchGraspRequest()
    req.parts_id = int(str(item.parts_id).strip("part_"))
    req.bin_name = item.bin_name
    req.gripper = item.ee_to_use
    req.is_updated = is_updated
    resp = self._search_grasp(req_search_grasp)

    pose_in_camera = geometry_msgs.PoseStamped()
    pose_in_camera.header.frame_id = "a_phoxi_m_sensor"
    pose_in_camera.pose.position = resp.pos3D[0]
    pose_in_camera.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(resp.rot3D[0].x, resp.rot3D[0].y, resp.rot3D[0].z))
    pose_in_bin.pose = tf_listener.transformPose(item.bin_name, pose_in_camera)
    pose_in_bin.pose.orientation = self.downward_orientation
    # TODO We should talk about how to use rotiqz which the two_finger approaches.

    return pose_in_bin

  def attempt_item(self, item, max_attempts = 5):
    """This function attempts to pick an item.
       It increases the item.attempts counter each time it does, 
       and sets item.fulfilled to True if item is delivered."""
    if "precision_gripper" in item.ee_to_use:
      robot_name = "a_bot"
      # end_effector_link = "a_bot_precision_gripper_tip_link"
    elif "robotiq_gripper" in item.ee_to_use:
      robot_name = "b_bot"
      # end_effector_link = "b_bot_robotiq_85_tip_link"
    elif item.ee_to_use == "suction":
      robot_name = "b_bot"
      # end_effector_link = "b_bot_suction_tool_tip_link"

    attempts = 0
    while attempts < max_attempts:
      attempts += 1
      item.attempts += 1
      rospy.loginfo("Attempting item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Attempt nr. " + str(item.attempts))
      
      # Attempt to pick the item
      # TODO: Get the position from vision
      pick_pose = geometry_msgs.msg.PoseStamped()
      pick_pose.header.frame_id = item.bin_name
      # pick_pose = self.get_item_pose(item)
      if item.ee_to_use == "suction":      # Orientation needs to be adjusted for suction tool
        pick_point_on_table = self.listener.transformPose("workspace_center", pick_pose).pose.position
        if pick_point_on_table.y > -.1:
          pick_pose.pose.orientation = self.suction_orientation_from_behind
        else:
          pick_pose.pose.orientation = self.suction_orientation_from_45
      else:   # Precision_gripper, robotiq_gripper
        pick_pose.pose.orientation = self.downward_orientation
        
      if item.ee_to_use == "precision_gripper_from_inside":
        gripper_command = "inner_gripper_from_inside"
      elif item.ee_to_use == "precision_gripper_from_outside":
        gripper_command = "inner_gripper_from_outside"
      elif item.ee_to_use == "suction":
        gripper_command = "suction"
      else:
        gripper_command = ""

      item_picked = self.pick(robot_name, pick_pose, 0.0, speed_fast = 0.3, speed_slow = 0.02, 
                        gripper_command=gripper_command, approach_height = 0.1)
      # TODO: Check grasp success via grasp width for robotiq gripper and vision for precision gripper
      # Suction is checked inside the pick function
      
      if not item_picked:
        rospy.logerr("Failed an attempt to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Reattempting. Current attempts: " + str(attempts))
        continue
      
      # Attempt to place the item
      place_pose = geometry_msgs.msg.PoseStamped()
      place_pose.header.frame_id = item.target_frame
      if item.ee_to_use == "suction":
        # This is inconvenient to set up because the trays are rotated. tray_2s are rotated 180 degrees relative to set_1_tray_1
        if item.set_number == 1:
          if item.target_frame[11] == '1':  # tray 1
            place_pose.pose.orientation = self.suction_orientation_from_45
          elif item.target_frame[11] == '2':  # tray 2
            place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2+pi))
        elif item.set_number == 2:
          if item.target_frame[11] == '1':  # tray 1
            place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))  # Facing right
          elif item.target_frame[11] == '2':  # tray 2
            place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/4+pi))
        elif item.set_number == 3:
          if item.target_frame[11] == '1':  # tray 1
            place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi*30/180))  # Facing towards camera
          elif item.target_frame[11] == '2':  # tray 2
            place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, +pi)) # Facing towards camera
        if not place_pose.pose.orientation.w and not place_pose.pose.orientation.x and not place_pose.pose.orientation.y:
          rospy.logerr("SOMETHING WENT WRONG, ORIENTATION IS NOT ASSIGNED")
      else:   # Precision_gripper, robotiq_gripper
        place_pose.pose.orientation = self.downward_orientation
      
      if item.ee_to_use == "suction":
        # Approach the place pose with controlled acceleration value
        place_pose.pose.position.z += .1
        self.move_lin(robot_name, place_pose, speed = 0.2, acceleration = 0.08, end_effector_link = "b_bot_suction_tool_tip_link")
        place_pose.pose.position.z -= .1
      
      self.place(robot_name, place_pose,grasp_height=item.dropoff_height,
                    speed_fast = 0.2, speed_slow = 0.02, gripper_command=gripper_command, approach_height = .1)

      # If successful
      self.fulfilled_items += 1
      item.fulfilled = True
      rospy.loginfo("Delivered item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")! Total items delivered: " + str(self.fulfilled_items))
      return True
    rospy.logerr("Was not able to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")! Total attempts: " + str(item.attempts))
    return False

  def prepare_robots(self):
    self.go_to_named_pose("back", "a_bot")
    self.go_to_named_pose("back", "c_bot")
    self.go_to_named_pose("screw_ready_back", "b_bot")
  
  def kitting_task(self):
    # Strategy:
    # - Start with suction tool + m4 tool equipped
    # - Pick all the screws in the current set and put them in the feeders, so they will be ready to be picked
    # - Go through all items that require suction, so the tool is equipped only once does not have to be re-equipped
    # - Go through items that are picked by the a_bot or b_bot grippers
    # - When the screws are ready, pick and place them
    # - Try to pick and return any excess screws
    # Ideally, we can fit all the trays into the scene.

    self.go_to_named_pose("back", "c_bot")
    self.go_to_named_pose("screw_ready_back", "b_bot")
    start_time = rospy.Time.now()
    time_limit = rospy.Duration(1140) # 19 minutes
    self.fulfilled_items = 0
    self.screws_done = dict()
    self.screws_placed = dict()
    self.screws_done["m4"] = (len(self.screws["m4"]) == 0)
    self.screws_done["m3"] = (len(self.screws["m3"]) == 0)
    self.screws_placed["m4"] = {1:0, 2:0, 3:0}
    self.screws_placed["m3"] = {1:0, 2:0, 3:0}

    # ==== 1. First, do an initial attempt on picking all items, ordered by the tool that they are picked with (screws, suction, precision_gripper, robotiq)

    # self.go_to_named_pose("home", "a_bot")
    # for item in self.screw_items:
    #   if rospy.is_shutdown():
    #     break
    #   self.pick_screw_from_bin_and_put_into_feeder(item)
    #   self.go_to_named_pose("home", "a_bot")
    # self.go_to_named_pose("back", "a_bot")
    screw_delivery_time = rospy.Time.now()

    for item in self.suction_items:
      if rospy.is_shutdown():
        break
      self.go_to_named_pose("suction_pick_ready", "b_bot")
      self.attempt_item(item, 10)
    self.do_change_tool_action("b_bot", equip=False, screw_size=50)   # 50 = suction tool
    self.go_to_named_pose("back", "b_bot")

    # for item in self.precision_gripper_items:
    #   if rospy.is_shutdown():
    #     break
    #   self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")
    #   self.attempt_item(item)
    # self.go_to_named_pose("back", "a_bot")

    # for item in self.robotiq_gripper_items:
    #   if rospy.is_shutdown():
    #     break
    #   self.attempt_item(item)
    # self.go_to_named_pose("back", "b_bot")

    # ==== 2. Second, loop through all the items that were not successfully picked on first try, and place the screws when they are assumed to be ready.
    all_done = False
    while not all_done and not rospy.is_shutdown():
      rospy.loginfo("Entering the loop to recover failed items and pick screws")

      # ### Pick the screws when they should be ready
      # if (rospy.Time.now() - screw_delivery_time) > rospy.Duration(5):
      #   for screw_size in [4,3]:
      #     if not self.screws_done["m"+str(screw_size)]:
      #       rospy.loginfo("Picking m" + str(screw_size) + " screws from feeder")
      #       self.go_to_named_pose("home", "c_bot")
      #       self.do_change_tool_action("c_bot", equip=True, screw_size=screw_size)
      #       for item in self.screws["m"+str(screw_size)]:
      #         if rospy.is_shutdown():
      #           break
      #         if self.pick_screw_from_feeder(screw_size, attempts=2):
      #           self.place_screw_in_tray(screw_size, item.set_number, self.screws_placed["m"+str(screw_size)][item.set_number]+1)
      #           self.screws_placed["m"+str(screw_size)][item.set_number] += 1
      #           self.fulfilled_items += 1
      #           item.fulfilled = True
      #         else:
      #           rospy.loginfo("Failed to pick an m" + str(screw_size) + " screw from the feeder")
      #           break
      #       # TODO: Return excess screws
      #       self.go_to_named_pose("screw_ready", "c_bot")
      #       self.do_change_tool_action("c_bot", equip=False, screw_size=screw_size)
      #       self.go_to_named_pose("back", "c_bot")
      
      # Check all items, find which groups are done, then reattempt those that are unfinished
      all_done = True      
      suction_done = True
      precision_gripper_done = True
      robotiq_gripper_done = True
      self.screws_done["m4"] = True
      self.screws_done["m3"] = True
      for item in self.ordered_items:
        if not item.fulfilled:
          rospy.loginfo("Found an undelivered item in the order: item nr. " + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")")
          all_done = False
          if "precision_gripper" in item.ee_to_use:
            precision_gripper_done = False
          elif "robotiq_gripper" in item.ee_to_use:
            robotiq_gripper_done = False
          elif item.ee_to_use == "suction":
            suction_done = False
          if item.part_id == 17:
            self.screws_done["m4"] = False
          elif item.part_id == 18:
            self.screws_done["m3"] = False

      # if not precision_gripper_done:
      #   rospy.loginfo("Reattempting the remaining precision gripper items")
      #   self.go_to_named_pose("taskboard_intermediate_pose", "a_bot")
      #   for item in self.precision_gripper_items:
      #     if rospy.is_shutdown():
      #             break
      #     self.attempt_item(item)
      #   self.go_to_named_pose("back", "a_bot")

      # if not robotiq_gripper_done:
      #   rospy.loginfo("Reattempting the remaining robotiq gripper items")
      #   self.go_to_named_pose("home", "b_bot")
      #   for item in self.robotiq_gripper_items:
      #     if rospy.is_shutdown():
      #             break
      #     self.attempt_item(item)
      #   self.go_to_named_pose("back", "b_bot")
      
      # if not suction_done:
      #   rospy.loginfo("Reattempting the remaining suction items")
      #   self.do_change_tool_action("b_bot", equip=True, screw_size=50)   # 50 = suction tool
      #   for item in self.suction_items:
      #     if rospy.is_shutdown():
      #             break
      #     if not item.fulfilled:
      #       self.go_to_named_pose("suction_pick_ready", "b_bot")
      #       self.attempt_item(item, 10)
      #   self.do_change_tool_action("b_bot", equip=False, screw_size=50)   # 50 = suction tool
      #   self.go_to_named_pose("back", "b_bot")

      if (rospy.Time.now()-start_time) > time_limit:
        rospy.logerr("Time limit reached! Breaking out of the loop.")
        break

    if all_done:
      rospy.loginfo("Completed the task.")
    else:
      rospy.loginfo("STOPPED THE TASK")
    return

  def pick_suction_test(self):
    res = self.suck(True)
    if not res:
      rospy.loginfo("failed")
      return
    
    start_time = rospy.get_rostime()
    suctioned = False
    while ((rospy.get_rostime().secs - start_time.secs) <= 10.0):
      if self._suctioned:
        rospy.loginfo("succeeded")
        return
    rospy.loginfo("failed")
  
  def place_suction_test(self):
    res = self.suck(False)
    if not res:
      rospy.loginfo("failed")
      return

    start_time = rospy.get_rostime()
    while ((rospy.get_rostime().secs - start_time.secs) <= 10.0):
      if not self._suctioned:
        rospy.loginfo("succeeded")
        return
    rospy.loginfo("failed")

if __name__ == '__main__':
  try:
    kit = KittingClass()
    i = 1
    while i:
      rospy.loginfo("Enter 1 to equip suction tool .")
      rospy.loginfo("Enter 11 to unequip suction tool .")
      rospy.loginfo("Enter 2 to move the robots home to starting positions.")
      rospy.loginfo("Enter 3 to pick with b_bot using suction tool.")
      rospy.loginfo("Enter 4 to place with b_bot using suction tool.")
      rospy.loginfo("Enter 60 to pick screw from bin with a_bot.")
      rospy.loginfo("Enter 61 to hand over screw from a_bot to c_bot.")
      rospy.loginfo("Enter 62 to move robots back.")
      rospy.loginfo("Enter START to start the task.")
      rospy.loginfo("Enter x to exit.")
      i = raw_input()
      if i == '1':
        kit.do_change_tool_action("b_bot", equip=True, screw_size=50)   # 50 = suction tool
      if i == '11':
        kit.do_change_tool_action("b_bot", equip=False, screw_size=50)   # 50 = suction tool
      if i == '2':
        kit.prepare_robots()
      elif i == '3':
        kit.pick_suction_test()
      elif i == '4':
        kit.place_suction_test()
      elif i == 'START' or i == 'start' or i == '5000':
        kit.kitting_task()
      elif i == "60":
        kit.go_to_named_pose("home", "a_bot")
        kit.pick_screw_precision_gripper("bin1_3", screw_size= 4)
      elif i == "61":
        kit.pick_screw_from_precision_gripper(screw_size=4)
      elif i == "62":
        kit.go_to_named_pose("screw_ready", "c_bot")
        kit.go_to_named_pose("back", "a_bot")
      elif i == "x":
        break
      
    
    ##### EXAMPLE 3 (How to look into a bin)
    # kit.view_bin("a_bot", "bin1_4")

    ##### EXAMPLE 4 (Look into all bins)
    # kit.go_to_named_pose("home", "a_bot")
    # kit.view_bin("a_bot", "bin1_1")
    # rospy.sleep(1)
    # kit.view_bin("a_bot", "bin1_2")
    # rospy.sleep(1)
    # kit.view_bin("a_bot", "bin1_3")
    # rospy.sleep(1)
    # kit.view_bin("a_bot", "bin1_4")
    # rospy.sleep(1)
    # kit.view_bin("a_bot", "bin1_5")
    # rospy.sleep(1)
    # kit.view_bin("a_bot", "bin2_1")
    # rospy.sleep(1)
    # kit.view_bin("a_bot", "bin2_2")
    # rospy.sleep(1)
    # kit.view_bin("a_bot", "bin2_3")
    # rospy.sleep(1)
    # kit.view_bin("a_bot", "bin2_4")
    # rospy.sleep(1)
    # kit.view_bin("a_bot", "bin3_1")
    # rospy.sleep(1)

    # kit.go_to_named_pose("home", "a_bot")
    # kit.go_to_named_pose("home", "b_bot")
    # kit.pick_screw_precision_gripper("bin1_3", screw_size= 4)
    # kit.pick_screw_from_precision_gripper(screw_size=4)

    ##### OLD CODE
    # kit.pick_and_place_demo()


    ##### WHAT THE REAL TASK SHOULD BE LIKE
    # rospy.loginfo("Press enter to start the task!")
    # raw_input()
    # kit.kitting_task()
    
    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
