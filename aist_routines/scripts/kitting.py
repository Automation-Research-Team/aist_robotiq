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
        "PartProps", "robot_name, camera_name, destination, grasp_offset, approach_offset, dropoff_height")
    _part_props = {
        4  : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_4",
                       -0.002, 0.10, 0.15),
        5  : PartProps("b_bot", "a_phoxi_m_camera", "tray_2_partition_6",
                       0.0, 0.10, 0.15),
        6  : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_3",
                       0.0, 0.10, 0.15),
        7  : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_2",
                       0.0, 0.10, 0.15),
        8  : PartProps("b_bot", "a_phoxi_m_camera", "tray_2_partition_1",
                       0.0, 0.10, 0.15),
        9  : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_4",
                       -0.001, 0.10, 0.15),
        10 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_7",
                       -0.001, 0.10, 0.15),
        11 : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_1",
                       0.0, 0.10, 0.15),
        12 : PartProps("b_bot", "a_phoxi_m_camera", "tray_2_partition_3",
                       0.0, 0.10, 0.15),
        13 : PartProps("b_bot", "a_phoxi_m_camera", "tray_1_partition_5",
                       0.0, 0.10, 0.15),
        14 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_2",
                       -0.001, 0.10, 0.15),
        15 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_5",
                       -0.001, 0.10, 0.15),
        16 : PartProps("a_bot", "a_phoxi_m_camera", "tray_2_partition_8",
                       -0.001, 0.10, 0.15),
        17 : PartProps("a_bot", "a_phoxi_m_camera", "skrewholder_1",
                       -0.001, 0.10, 0.15),
        18 : PartProps("a_bot", "a_phoxi_m_camera", "skrewholder_2",
                       -0.001, 0.10, 0.15),
    }
    BinProps = collections.namedtuple("BinProps",
                                      "bin_id, part_id, part_props")

    def __init__(self):
        super(KittingRoutines, self).__init__()
        self._bins = [
            "bin_1_part_5",
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
        #self.go_to_named_pose("home", "all_bots")

    @property
    def nbins(self):
        return len(self._bins)

    def item(self, bin):
        return self._items[bin]

    ###----- main procedure
    def kitting_task(self):
        self.go_to_named_pose("back", "all_bots")
        for bin in self._bins:
            if rospy.is_shutdown():
                break
            self.attempt_bin(bin, 1)
        self.go_to_named_pose("home", "all_bots")

    def attempt_bin(self, bin, max_attempts=5, marker_lifetime=0):
        item  = self._items[bin]
        props = item.part_props

        # If using a different robot from the former, move it back to home.
        if self._former_robot_name != None and \
           self._former_robot_name != props.robot_name:
            self.go_to_named_pose("back", self._former_robot_name)
        self._former_robot_name = props.robot_name

        # Move to 0.15m above the bin if the camera is mounted on the robot, or
        # move to 0.15m above the destination oterhwise.
        if self._is_eye_on_hand(props.robot_name, props.camera_name):
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

            if self.pick(props.robot_name, pick_poses[i],
                         grasp_offset=props.grasp_offset,
                         approach_offset=props.approach_offset):
                self.place_at_frame(props.robot_name, props.destination,
                                    (0, 0, props.dropoff_height))
                return True
            else:
                self.release(props.robot_name)

        if self._is_eye_on_hand(props.robot_name, props.camera_name):
            self.go_to_frame(props.robot_name, bin, (0, 0, 0.15))
        else:
            self.go_to_frame(props.robot_name, props.destination, (0, 0, 0.15))
        return False

    def _is_eye_on_hand(self, robot_name, camera_name):
        return camera_name == robot_name + "_camera"


if __name__ == '__main__':
    with KittingRoutines() as kitting:
        while not rospy.is_shutdown():
            print("============ Kitting procedures ============ ")
            print("  g: Create a backgroud image")
            print("  m: Create a mask image")
            print("  s: Search graspabilities")
            print("  a: Attempt pick and place")
            print("  k: Do kitting task")
            print("  H: Move all robots to home")
            print("  B: Move all robots to back")
            print("  q: Quit")

            try:
                key = raw_input(">> ")
                kitting.delete_all_markers()
                if key == 'q':
                    break
                elif key == 'h':
                    kitting.go_to_named_pose("home", "all_bots")
                elif key == 'b':
                    kitting.go_to_named_pose("back", "all_bots")
                elif key == 'g':
                    kitting.create_background_image("a_phoxi_m_camera")
                elif key == 'm':
                    kitting.create_mask_image("a_phoxi_m_camera", kitting.nbins)
                elif key == 's':
                    item  = kitting.item(raw_input("  bin name? "))
                    props = item.part_props
                    kitting.search_graspability(props.robot_name,
                                                props.camera_name,
                                                item.part_id, item.bin_id, 0)
                elif key == 'a':
                    bin = raw_input("  bin name? ")
                    kitting.attempt_bin(bin, 5, 0)
                elif key == 'k':
                    kitting.kitting_task()
            except Exception as e:
                print(e)
