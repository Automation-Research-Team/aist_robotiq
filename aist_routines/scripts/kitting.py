#!/usr/bin/env python

import os, csv, copy, time, datetime, re, collections
import rospy, rospkg

from aist_routines.base import AISTBaseRoutines

######################################################################
#  global variables                                                  #
######################################################################
rp = rospkg.RosPack()

ts = time.time()
start_date_time = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
number_of_attempted = 1

######################################################################
#  class KittingRoutines                                             #
######################################################################
class KittingRoutines(AISTBaseRoutines):
    """Implements kitting routines for aist robot system."""
    PartProps = collections.namedtuple(
        "PartProps", "robot_name, camera_name, destination, grasp_offset, dropoff_height")
    _part_props = {
        4  : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_4",
                       0.0, 0.15),
        5  : PartProps("b_bot", "a_phoxi_m_camera", "tray_2_partition_6",
                       0.0, 0.15),
        6  : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_3",
                       0.0, 0.15),
        7  : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_2",
                       0.0, 0.15),
        8  : PartProps("b_bot", "a_phoxi_m_camera", "tray_2_partition_1",
                       0.0, 0.15),
        9  : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_4",
                       -0.001, 0.15),
        10 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_7",
                       -0.001, 0.15),
        11 : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_1",
                       0.0, 0.15),
        12 : PartProps("b_bot", "a_phoxi_m_camera", "tray_2_partition_3",
                       0.0, 0.15),
        13 : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_5",
                       0.0, 0.15),
        14 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_2",
                       -0.001, 0.15),
        15 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_5",
                       -0.001, 0.15),
        16 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_8",
                       -0.001, 0.15),
        17 : PartProps("a_bot", "a_phoxi_m_camera", "skrewholder_1",
                       -0.001, 0.15),
        18 : PartProps("a_bot", "a_phoxi_m_camera", "skrewholder_2",
                       -0.001, 0.15),
    }
    BinProps = collections.namedtuple("BinProps",
                                      "bin_id, part_id, part_props")

    def __init__(self):
        super(KittingRoutines, self).__init__()
        self._bins = [
            "bin_1_part_15",
            "bin_1_part_16",
            "bin_1_part_17",
            "bin_2_part_4",
            "bin_2_part_7",
            "bin_2_part_8",
        ]

        # Assign part information to each bin.
        self._items = {}
        for bin_id, bin in enumerate(self._bins, 1):
            part_id = int(re.search("[0-9]+$", bin).group())
            self._items[bin] = KittingRoutines.BinProps(
                bin_id, part_id, KittingRoutines._part_props[part_id])

        self._former_robot_name = None
        self.go_to_named_pose("home", "all_bots")

    ###----- main procedure
    def kitting_task(self):
        self.go_to_named_pose("back", "all_bots")
        for bin in self._bins:
            if rospy.is_shutdown():
                break
            self.attempt_bin(bin, 1)
        self.go_to_named_pose("home", "all_bots")

    def attempt_bin(self, bin, max_attempts=5, marker_lifetime=60):
        item  = self._items[bin]
        props = item.part_props

        # If using a different robot from the former, move it back to home.
        if _former_robot_name != None and \
           _former_robot_name != props.robot_name:
            self.go_to_named_pose("back" _former_robot_name)
        _former_robot_name = props.robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot, or
        # move to 0.15m above the destination oterhwise.
        if self._eye_in_hand(props.robot_name, props.camera_name):
            self.go_to_frame(props.robot_name, bin, (0, 0, 0.15))
        else:
            self.go_to_frame(props.robot_name, props.destination, (0, 0, 0.15))

        # Search for graspabilities.
        (pick_poses, rotipz, gscore, success) \
            = self.search_graspability(props.robot_name, props.camera_name,
                                       item.part_id, item.bin_id,
                                       marker_lifetime)
        if not success:
            return False

        # Attempt to pick the item.
        for i in range(max_attempts):
            if rospy.is_shutdown():
                break

            if self.pick(props.robot_name, pick_poses[i], props.grasp_offset):
                self.place_at_frame(props.robot_name, props.detination,
                                    (0, 0, props.dropoff_height))
                return True

        return False

    def _eye_in_hand(robot_name, camera_name):
        return camera_name == robot_name + "_camera"


if __name__ == '__main__':
    with KittingClass(vision_algo="fge") as kitting:
        while not rospy.is_shutdown():
            rospy.loginfo("")
            rospy.loginfo("1: to go to home all robots.")
            rospy.loginfo("322: Bins with b_bot")
            rospy.loginfo("332: Bin corners with b_bot")
            rospy.loginfo("Enter 71, 72... to test phoxi on item 1, 2...")
            rospy.loginfo("Enter START to start the task.")
            rospy.loginfo("Enter x to exit.")

            i = raw_input()
            if i == '1':
                kitting.go_to_named_pose("home", "b_bot")
            elif i == '322':
                kitting.bin_calibration(robot_name="b_bot")
            elif i == '332':
                kitting.bin_corner_calibration(robot_name="b_bot")
            elif i in ["71", "72", "73", "74", "75", "76", "77", "78", "79"]:
                item = kitting.ordered_items[int(i)-71]
                rospy.loginfo("Checking for item id " + str(item.part_id) + " in " + item.bin_name)
                kitting.get_grasp_candidates_from_phoxi(item, True)
                rospy.loginfo("Grasp candidates of item " + str(item.part_id))
                rospy.loginfo(kitting.grasp_candidates[item.part_id]["position"])
                try:
                    kitting.publish_marker(kitting.grasp_candidates[item.part_id]["position"].pop(0), "aist_vision_result")
                except IndexError:
                    pass
            elif i == 'START' or i == 'start':
                kitting.kitting_task()
            elif i == 'x':
                break
        print("================ Done!")
