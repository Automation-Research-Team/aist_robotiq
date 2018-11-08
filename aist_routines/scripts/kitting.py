#!/usr/bin/env python

from math import pi
import random
import os
import csv

import rospy
import rospkg
import tf_conversions
import std_msgs.msg
import geometry_msgs.msg

from aist_routines.base import AISTBaseRoutines
import o2as_msgs.msg


rp = rospkg.RosPack()

class kitting_order_entry():
    """
    Object that tracks if its order was fulfilled, and the number of attempts spent on it.
    """
    def __init__(self, part_id, set_number, number_in_set, bin_name, target_frame, ee_to_use, item_name):
        self.part_id = part_id  # The part id
        self.set_number = set_number
        self.number_in_set = number_in_set
        self.bin_name = bin_name
        self.target_frame = target_frame
        self.ee_to_use = ee_to_use
        self.item_name = item_name

        self.attempts = 0
        self.fulfilled = False
        self.in_feeder = False


class KittingClass(AISTBaseRoutines):
    """Implements kitting routines for aist robot system."""

    def __init__(self):
        """Initialize class object."""
        super(KittingClass, self).__init__()
        self.initial_setup()
        rospy.loginfo("Kitting class is staring up!")

    def read_order_file(self):
        """
        Read in the order file, return kitting_list and order_entry_list.

        kitting_list is a list of lists with only the part IDs that are ordered.
        order_entry_list is a list of kitting_entry_item objects.
        """
        order_entry_list = []
        kitting_list = []
        kitting_list.append([])
        kitting_list.append([])
        kitting_list.append([])

        with open(os.path.join(rp.get_path("aist_scene_description"), "config", "kitting_order_file.csv"), 'r') as f:
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
                                                            item_name=data[4]))
        return kitting_list, order_entry_list

    def initial_setup(self):
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

        self.part_bin_list = {
            "part_4" : "bin_2_part_4",
            "part_5" : "bin_2_part_5",
            "part_6" : "bin_3_part_6",
            "part_7" : "bin_3_part_7",
            "part_8" : "bin_2_part_8",
            "part_9" : "bin_1_part_9",
            "part_10" : "bin_1_part_10",
            "part_11" : "bin_2_part_11",
            "part_12" : "bin_1_part_12",
            "part_13" : "bin_2_part_13",
            "part_14" : "bin_1_part_14",
            "part_15" : "bin_1_part_15",
            "part_16" : "bin_1_part_16",
            "part_17" : "bin_1_part_17",
            "part_18" : "bin_1_part_18"
        }

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
            "part_18": "tray_2_this_is_a_screw_so_should_be_ignored"
        }

        self.grasp_strategy = {
            "part_4" : "suction",
            "part_5" : "suction",
            "part_6" : "robotiq_gripper",
            "part_7" : "suction",
            "part_8" : "suction",
            "part_9" : "precision_gripper_from_inside",
            "part_10": "precision_gripper_from_inside",
            "part_11": "suction",
            "part_12": "suction",
            "part_13": "suction",
            "part_14": "precision_gripper_from_outside",
            "part_15": "precision_gripper_from_inside",
            "part_16": "precision_gripper_from_inside",
            "part_17": "precision_gripper_from_outside",
            "part_18": "precision_gripper_from_outside"
        }

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

        while True:
            rospy.loginfo("Enter 1 to read order file.")
            rospy.loginfo("Enter x to exit.")

            i = raw_input()
            if i == '1':
                order_list_raw, ordered_items = kit.read_order_file()
                rospy.loginfo("Received order list:")
                rospy.loginfo(order_list_raw)
            elif i == 'x':
                break
        print("================ Done!")
    except rospy.ROSInterruptException:
        pass
