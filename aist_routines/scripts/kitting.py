#!/usr/bin/env python

from math import pi
import random

import rospy
import tf_conversions
import std_msgs.msg
import geometry_msgs.msg

from aist_routines.base import AISTBaseRoutines
import o2as_msgs.msg


class kitting_order_entry(object):
    """Definition class of target item"""
    def __init__(self, part_id, bin_name, ee_to_use):
        self.part_id = part_id
        self.bin_name = bin_name
        self.ee_to_use = ee_to_use


class KittingClass(AISTBaseRoutines):
    """Implements kitting routines for aist robot system."""

    def __init__(self):
        """Initialize class object."""
        super(KittingClass, self).__init__()
        self.initialize_parameters()
        rospy.loginfo("Kitting class is staring up!")

    def initialize_parameters(self):
        """Initialize class parameters."""
        self.use_real_robot = rospy.get_param("use_real_robot", False)

        # Bin sizes to use random picking.
        # `width` is defined as the size in the x-axis direction.
        # `height` is defined as the size in the y-axis direction.
        # The size range is equaled or longer than 0.
        self.bin_1_width = 0.128
        self.bin_1_length = 0.125
        self.bin_2_width = 0.201
        self.bin_2_length = 0.112
        self.bin_3_width = 0.285
        self.bin_3_length = 0.192

        self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))

    def get_random_pose_in_bin(self, item):
        """Get item's random pose in parts bin."""
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = item.bin_name

        if "bin1" in item.bin_name:
            bin_width = self.bin_1_width
            bin_length = self.bin_1_length
        elif "bin2" in item.bin_name:
            bin_width = self.bin_2_width
            bin_length = self.bin_2_length
        elif "bin3" in item.bin_name:
            bin_width = self.bin_3_width
            bin_length = self.bin_3_length

        pick_pose.pose.position.x += -bin_width/2 + random.random()*bin_width
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
        pick_pose.pose.position.y += -bin_length/2 + random.random()*bin_length

        return pick_pose

if __name__ == '__main__':

    rospy.init_node("Kitting")

    try:
        kit = KittingClass()

        while not rospy.is_shutdown():
            rospy.loginfo("x: Exit")

            i = raw_input()
            if i == '1':
                pass
            elif i == 'x':
                break
        print("================ Done!")
    except rospy.ROSInterruptException:
        pass
