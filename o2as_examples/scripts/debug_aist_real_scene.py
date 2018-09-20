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

class DebugAistRealRobotClass(O2ASBaseRoutines):
    """
    This contains the routine used to run the kitting task. See base.py for shared convenience functions.
    """
    def __init__(self):

        super(DebugAistRealRobotClass, self).__init__()
        
        # params
        self.speed_fast = 0.05
        self.speed_slow = 0.05
        self.bin_list = rospy.get_param("bin_list")
        self.tray_partition_list = rospy.get_param("tray_partition_list")
        # self.gripper_id = rospy.get_param('gripper_id')
        # self.tray_id = rospy.get_param('tray_id')
        
        # services
        # self._suction = rospy.ServiceProxy("o2as_usb_relay/set_power", SetPower)
        # self._search_grasp = rospy.ServiceProxy("search_grasp", SearchGrasp)

        # self.set_up_item_parameters()
        # rospy.sleep(.5)

    def set_up_item_parameters(self):
        self.item_names = []
        self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        # 

    def go_to_each_bin(self):

        try:
            self.go_to_named_pose("home", "b_bot", self.speed_fast)

            for i in range(10):
                goal_pose = geometry_msgs.msg.PoseStamped()
                goal_pose.header.frame_id = self.bin_list[i]["bin_name"]
                goal_pose.pose.position.z = 0.15
                goal_pose.pose.orientation = copy.deepcopy(self.downward_orientation)
                self.move_lin("b_bot", goal_pose, self.speed_slow, "b_bot_dual_suction_gripper_pad_link")
                raw_input()

        except rospy.ROSInterruptException:
            return

    def go_to_each_tray(self):
        try:

            self.go_to_named_pose("home", "b_bot", self.speed_fast)

            for i in range(10):
                if self.tray_partition_list[i] == "NONE":
                    continue
                goal_pose = geometry_msgs.msg.PoseStamped()
                goal_pose.header.frame_id = self.tray_partition_list[i]
                goal_pose.pose.position.z = 0.05
                goal_pose.pose.orientation = copy.deepcopy(self.downward_orientation)
                self.move_lin("b_bot", goal_pose, self.speed_slow, "b_bot_dual_suction_gripper_pad_link")
                raw_input()

        except rospy.ROSInterruptException:
            return


if __name__ == '__main__':

  try:
    debug = DebugAistRealRobotClass()
    debug.set_up_item_parameters()
    debug.go_to_each_bin()
    # debug.go_to_each_tray()

    print "============ Done!"
  except rospy.ROSInterruptException:
    exit(0)
