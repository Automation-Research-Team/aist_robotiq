#!/usr/bin/env python

from math import pi
import random
import os
import csv
import copy
import time
import datetime

import rospy
import rospkg
import tf_conversions
import std_msgs.msg
import geometry_msgs.msg
import actionlib

from aist_routines.base import AISTBaseRoutines
import o2as_msgs.msg
import aist_graspability.msg


rp = rospkg.RosPack()

ts = time.time()
start_date_time = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')
number_of_attempted = 1

class kitting_order_entry():
    """
    Object that tracks if its order was fulfilled, and the number of attempts spent on it.
    """
    def __init__(self, part_id, set_number, number_in_set, bin_name, target_frame, ee_to_use, item_name, dropoff_height):
        self.part_id = part_id  # The part id
        self.set_number = set_number
        self.number_in_set = number_in_set
        self.bin_name = bin_name
        self.target_frame = target_frame
        self.ee_to_use = ee_to_use
        self.item_name = item_name
        self.dropoff_height = dropoff_height

        self.attempts = 0
        self.fulfilled = False
        self.in_feeder = False

def clamp(n, minn, maxn):
    """Constrain a number n to the interval [minn, maxn]"""
    return min(max(n, minn), maxn)

class KittingClass(AISTBaseRoutines):
    """Implements kitting routines for aist robot system."""

    def __init__(self, vision_algo="fge"):
        """Initialize class object."""
        super(KittingClass, self).__init__()
        self.vision_algo = vision_algo
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
                                                            item_name=data[4],
                                                            dropoff_height=self.dropoff_heights["part_" + data[2]]))
        return kitting_list, order_entry_list

    def initial_setup(self):
        """Initialize class parameters."""
        self.use_real_robot = rospy.get_param("use_real_robot", False)
        self.is_aist_experiment = rospy.get_param("is_aist_experiment", True)

        self._search_grasp_from_phoxi_client = actionlib.SimpleActionClient(
                                                    "aist_graspability/search_grasp_from_phoxi",
                                                    aist_graspability.msg.SearchGraspFromPhoxiAction
                                                )

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
            "part_a" :"bin_2_part_a",
            "part_4" :"bin_2_part_4",
            "part_7" :"bin_2_part_7",
            "part_11":"bin_2_part_11",
            "part_b" :"bin_2_part_b",
            "part_13":"bin_1_part_13",
            "part_8" :"bin_1_part_8",
            "part_12":"bin_1_part_12",
            "part_5" :"bin_1_part_5",
            "part_c" :"bin_1_part_c",
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

        # How high the end effector should hover over the tray when delivering the item
        self.dropoff_heights = {
            "part_4" : 0.15,
            "part_5" : 0.15,
            "part_6" : 0.15,
            "part_7" : 0.15,
            "part_8" : 0.15,
            "part_9" : 0.15,
            "part_10": 0.15,
            "part_11": 0.15,
            "part_12": 0.15,
            "part_13": 0.15,
            "part_14": 0.15,
            "part_15": 0.15,
            "part_16": 0.15,
            "part_17": 0.15,
            "part_18": 0.15
        }

        # How many candidates should we get from phoxi.
        self.max_candidates_from_phoxi = 3
        self.grasp_candidates = {
            4 : {
                "position" : []
            },
            5 : {
                "position" : []
            },
            6 : {
                "position" : []
            },
            7 : {
                "position" : []
            },
            8 : {
                "position" : []
            },
            9 : {
                "position" : []
            },
            10: {
                "position" : []
            },
            11: {
                "position" : []
            },
            12: {
                "position" : []
            },
            13: {
                "position" : []
            },
            14: {
                "position" : []
            },
            15: {
                "position" : []
            },
            16: {
                "position" : []
            },
            17: {
                "position" : []
            },
            18:{
                "position" : []
            }
        }

        self.downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
        self.downward_orientation2 = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, -pi/2))

        self.order_list_raw, self.ordered_items = self.read_order_file()
        rospy.loginfo("Received order list:")
        rospy.loginfo(self.order_list_raw)
        self.suction_items = []
        for order_item in self.ordered_items:
            if order_item.ee_to_use == "suction":
                rospy.loginfo("Appended item nr." + str(order_item.number_in_set) + " from set " + str(order_item.set_number) + " (part ID:" + str(order_item.part_id) + ") to list of suction items")
                self.suction_items.append(order_item)

    def attempt_item(self, item, max_attempts = 5):
        """
        This function attempts to pick an item.

        It increases the item.attempts counter each time it does,
        and sets item.fulfilled to True if item is delivered.
        """
        if item.ee_to_use == "suction":
            robot_name = "b_bot"

        if item.fulfilled:
            rospy.logerr("This item is already fulfilled. Something is going wrong.")
            return False

        # Go to preparatory pose
        if item.ee_to_use == "suction":
            rospy.loginfo("Going to preparatory pose before picking from bins")
            bin_center = geometry_msgs.msg.PoseStamped()
            bin_center.header.frame_id = item.bin_name
            bin_center.pose.orientation.w = 1.0
            bin_center_on_table = self.listener.transformPose("workspace_center", bin_center).pose.position
            if bin_center_on_table.y > .1:
                self.go_to_named_pose("suction_ready_right_bins", "b_bot", speed=1.0, acceleration=1.0)
            else:
                self.go_to_named_pose("suction_ready_left_bins", "b_bot", speed=1.0, acceleration=1.0)

        attempts = 0
        while attempts < max_attempts and not rospy.is_shutdown():
            attempts += 1
            item.attempts += 1
            rospy.loginfo("=== Attempting item nr." + str(item.number_in_set) +
                          " from set " + str(item.set_number) +
                          " (part ID:" + str(item.part_id) + "). Attempt nr. " + str(item.attempts))
            # Get the pick_pose for the item, either random or from vision
            pick_pose = self.get_random_pose_in_bin(item)
            if self.grasp_candidates[item.part_id]["position"]:
                rospy.loginfo("Use candidate pose estimating by vision.")
                pick_pose = self.grasp_candidates[item.part_id]["position"].pop(0)
                rospy.loginfo("Pick pose in bin:")
                rospy.loginfo(pick_pose)
            raw_input("Press `Enter` to pick item.")
            # Magic offset go more down 5mm (since 2018/11/19) TODO: We need force sensor to get high accuracy.
            pick_pose.pose.position.z -= 0.003
            pick_pose.pose.orientation = self.downward_orientation
            approach_height = 0.15
            if item.ee_to_use == "suction":
                gripper_command = "suction"
                approach_height = .15
            else:
                gripper_command = ""
            # pick_pose = self.make_pose_safe_for_bin(pick_pose, item)
            self.publish_marker(pick_pose, "aist_vision_result")
            item_picked = self.pick(robot_name, pick_pose, grasp_height=0.0, speed_fast=.05, speed_slow=.05,
                                                gripper_command=gripper_command, approach_height=approach_height, timeout=10.0)
            if not self.use_real_robot:
                item_picked = True
            if not item_picked:
                rospy.logerr("Failed an attempt to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Reattempting. Current attempts: " + str(attempts))
                if item.ee_to_use == "suction":
                    self.suck(turn_suction_on=False, eject=False)
                continue

            raw_input("Press `Enter` to place item.")

            if item.ee_to_use == "suction":
                rospy.loginfo("Going to circumstantial pose after picking from bins")
                bin_center = geometry_msgs.msg.PoseStamped()
                bin_center.header.frame_id = item.bin_name
                bin_center.pose.orientation.w = 1.0
                bin_center_on_table = self.listener.transformPose("workspace_center", bin_center).pose.position
                if bin_center_on_table.y > .1:
                    self.go_to_named_pose("suction_ready_right_bins", "b_bot", speed=1.0, acceleration=1.0)
                else:
                    self.go_to_named_pose("suction_ready_left_bins", "b_bot", speed=1.0, acceleration=1.0)

            # Attempt to place the item
            place_pose = geometry_msgs.msg.PoseStamped()
            if self.is_aist_experiment:
                place_pose.header.frame_id = "place_bin"
            else:
                place_pose.header.frame_id = item.target_frame
            place_pose.pose.orientation = self.downward_orientation2
            approach_height = .15
            if item.ee_to_use == "suction":
                self.go_to_named_pose("suction_ready_above_place_bin", "b_bot", speed=1.0, acceleration=1.0)
            self.place(robot_name, place_pose, place_height=item.dropoff_height,
                                        speed_fast=1.0, speed_slow=0.05, gripper_command=gripper_command, approach_height=approach_height)
            if item.ee_to_use == "suction":
                self.go_to_named_pose("suction_ready_above_place_bin", "b_bot", speed=1.0, acceleration=1.0)
            # If successful
            self.fulfilled_items += 1
            item.fulfilled = True
            rospy.loginfo("Delivered item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")! Total items delivered: " + str(self.fulfilled_items))
            return True

        rospy.logerr("Was not able to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")! Total attempts: " + str(item.attempts))
        if item.ee_to_use == "suction":
            self.go_to_named_pose("home", "b_bot")
        return False

    def get_grasp_candidates_from_phoxi(self, item, take_new_image):
        """Get item's pose in parts bin using phoxi."""

        goal = aist_graspability.msg.SearchGraspFromPhoxiGoal()
        goal.take_new_image = take_new_image
        goal.part_id = item.part_id
        goal.bin_name = item.bin_name
        goal.scene_path = os.path.join(rp.get_path("aist_graspability"), "data", start_date_time + "_part_" + str(item.part_id) + "_attempt_" + str(number_of_attempted) + ".tif")
        goal.mask_path = os.path.join(rp.get_path("aist_graspability"), "data", "imr3.png")
        goal.algorithm = self.vision_algo

        if "precition_gripper" in item.ee_to_use:
            if "inside" in item.ee_to_use:
                goal.gripper_type = "inner"
            elif "outside" in item.ee_to_use:
                goal.gripper_type = "two_finger"
        elif "suction" in item.ee_to_use:
            goal.gripper_type = "suction"
        elif "robotiq":
            goal.gripper_type = "two_finger"
        else:
            rospy.logerr("Gripper type is undefined.")
            return False

        try:
            self._search_grasp_from_phoxi_client.send_goal(goal)
            self._search_grasp_from_phoxi_client.wait_for_result() # Not recommended because it may take too long time to get vision result.
            # self._search_grasp_from_phoxi_client.wait_for_result(rospy.Duration(10.0))
            resp_search_grasp = self._search_grasp_from_phoxi_client.get_result()
        except:
            rospy.logerr("Could not get grasp from Phoxi with action client exception.")
            return False

        if resp_search_grasp is None or not resp_search_grasp.success:
            rospy.logerr("Could not get grasp from Phoxi with no candidates.")
            return False

        poses_in_bin = []
        pose0 = geometry_msgs.msg.PointStamped()
        pose0.header.frame_id = "a_phoxi_m_sensor"
        number_of_pose_candidates = min(self.max_candidates_from_phoxi, resp_search_grasp.result_num)
        for i in range(number_of_pose_candidates):
            object_position = copy.deepcopy(pose0)
            object_position.point = geometry_msgs.msg.Point(
                resp_search_grasp.pos3D[i].x,
                resp_search_grasp.pos3D[i].y,
                resp_search_grasp.pos3D[i].z)
            rospy.logdebug("\nGrasp point in %s: (x, y, z) = (%f, %f, %f)",
                object_position.header.frame_id,
                object_position.point.x,
                object_position.point.y,
                object_position.point.z)
            obj_pose_in_camera = geometry_msgs.msg.PoseStamped()
            obj_pose_in_camera.header = object_position.header
            obj_pose_in_camera.pose.position = object_position.point
            obj_pose_in_camera.pose.orientation.w = 1.0
            pose_in_bin = self.listener.transformPose(item.bin_name, obj_pose_in_camera)
            pose_in_bin.pose.orientation = self.downward_orientation
            if item.ee_to_use in ["precision_gripper_from_outside", "robotiq_gripper"]:
                pose_in_bin.pose.orientation = geometry_msgs.msg.Quaternion(
                    *tf_conversions.transformations.quaternion_from_euler(0, pi/2, -resp_search_grasp.rotipz[i]))
            poses_in_bin.append(pose_in_bin)
        rospy.loginfo("Calculated " + str(number_of_pose_candidates) + " candidates for item nr. " + str(item.part_id) + " in bin " + str(item.bin_name))
        rospy.logdebug(poses_in_bin)

        self.grasp_candidates[item.part_id]["position"] = copy.deepcopy(poses_in_bin)

        return

    def get_random_pose_in_bin(self, item):
        """Get item's random pose in parts bin."""
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = item.bin_name

        if "bin_1" in item.bin_name:
            bin_width = self.bin_1_width
            bin_length = self.bin_1_length
        elif "bin_2" in item.bin_name:
            bin_width = self.bin_2_width
            bin_length = self.bin_2_length
        elif "bin_3" in item.bin_name:
            bin_width = self.bin_3_width
            bin_length = self.bin_3_length

        pick_pose.pose.position.x = -bin_width/2 + random.random()*bin_width
        pick_pose.pose.position.y = -bin_length/2 + random.random()*bin_length
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))

        return pick_pose

    def make_pose_safe_for_bin(self, pick_pose, item):
        """ This makes sure that the pick_pose is not outside the bin or would cause a collision."""
        if "bin_1" in item.bin_name:
            bin_width = self.bin_1_width
            bin_length = self.bin_1_length
            thresh_x = .020
            thresh_y = .020
        elif "bin_2" in item.bin_name:
            bin_width = self.bin_2_width
            bin_length = self.bin_2_length
            thresh_x = .025
            thresh_y = .030
        elif "bin_3" in item.bin_name:
            bin_width = self.bin_3_width
            bin_length = self.bin_3_length
            thresh_x = .020
            thresh_y = .020

        rospy.loginfo("threshould: " + str(thresh_x) + ", " + str(thresh_y))

        safe_pose = copy.deepcopy(pick_pose)
        safe_pose.pose.position.x = clamp(pick_pose.pose.position.x, -bin_width/2 + thresh_x, bin_width/2 - thresh_x)
        safe_pose.pose.position.y = clamp(pick_pose.pose.position.y, -bin_length/2 + thresh_y, bin_length/2 - thresh_y)
        safe_pose.pose.position.z = clamp(pick_pose.pose.position.z, 0, 0.1)

        if safe_pose.pose.position.x != pick_pose.pose.position.x or safe_pose.pose.position.y != pick_pose.pose.position.y:
            rospy.loginfo("Pose was adjusted in make_pose_safe_for_bin. Before: " +
                                        str(pick_pose.pose.position.x) + ", " +
                                        str(pick_pose.pose.position.y) + ", " +
                                        str(pick_pose.pose.position.z) + ". After: " +
                                        str(safe_pose.pose.position.x) + ", " +
                                        str(safe_pose.pose.position.y) + ", " +
                                        str(safe_pose.pose.position.z) + ".")

        #TODO: Adjust the gripper orientation when close to the border
        return safe_pose

    ###----- main procedure
    def kitting_task(self):
        global number_of_attempted

        self.fulfilled_items = 0
        for item in self.suction_items:
            if rospy.is_shutdown():
                break
            self.get_grasp_candidates_from_phoxi(item, True)
            self.attempt_item(item, 1)
            self.grasp_candidates[item.part_id]["position"] = []
            number_of_attempted += 1
        self.go_to_named_pose("home", "b_bot")
        rospy.loginfo("==== Done with first suction pass")


if __name__ == '__main__':


    try:
        kit = KittingClass(vision_algo="fge")

        while True:
            rospy.loginfo("Enter 1 to go to home all robots.")
            rospy.loginfo("Enter 71, 72... to test phoxi on item 1, 2...")
            rospy.loginfo("Enter START to start the task.")
            rospy.loginfo("Enter x to exit.")

            i = raw_input()
            if i == '1':
                kit.go_to_named_pose("home", "b_bot")
            elif i in ["71", "72", "73", "74", "75", "76", "77", "78", "79"]:
                item = kit.ordered_items[int(i)-71]
                rospy.loginfo("Checking for item id " + str(item.part_id) + " in " + item.bin_name)
                kit.get_grasp_candidates_from_phoxi(item, True)
                rospy.loginfo("Grasp candidates of item " + str(item.part_id))
                rospy.loginfo(kit.grasp_candidates[item.part_id]["position"])
                kit.publish_marker(kit.grasp_candidates[item.part_id]["position"][0], "aist_vision_result")
            elif i == 'START' or i == 'start':
                kit.kitting_task()
            elif i == 'x':
                break
        print("================ Done!")
    except rospy.ROSInterruptException:
        pass
