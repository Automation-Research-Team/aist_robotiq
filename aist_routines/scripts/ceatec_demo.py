#!/usr/bin/env python

import os, csv, copy, time, datetime, re, collections
import rospy, rospkg, rosparam
import aist_routines.base as base
from aist_routines.fetch import FetchRoutines
from aist_msgs           import msg as amsg

######################################################################
#  global variables                                                  #
######################################################################
rp = rospkg.RosPack()

ts = time.time()
start_date_time = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
number_of_attempted = 1

######################################################################
#  class CeatecDemoRoutines                                          #
######################################################################
class CeatecDemoRoutines(FetchRoutines):
    """Implements kitting routines for aist robot system."""

    BinProps = collections.namedtuple("BinProps", "bin_id part_id part_props")

    def __init__(self):
        super(CeatecDemoRoutines, self).__init__()

        rospack = rospkg.RosPack()
        d = rosparam.load_file(rospack.get_path("aist_routines") +
                               "/config/ceatec_demo.yaml")[0][0]
        self._bins = d["bins"]
        item_props = base.paramtuples(d["item_props"])

        # Assign part information to each bin.
        self._items = {}
        for bin_id, bin in enumerate(self._bins, 1):
            part_id = int(re.search("[0-9]+$", bin).group())
            self._items[bin] \
                = CeatecDemoRoutines.BinProps(bin_id, part_id,
                                              item_props[part_id])

        self._fail_poses = []
        #self.go_to_named_pose("home", "all_bots")

    @property
    def nbins(self):
        return len(self._bins)

    def item(self, item_name):
        return self._items[item_name]

    ###----- main procedure
    def run(self):
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

        # Move to 0.15m above the bin if the camera is mounted on the robot.
        if self._is_eye_on_hand(props.robot_name, props.camera_name):
            self.go_to_frame(props.robot_name, bin, (0, 0, 0.15))

        # Search for graspabilities.
        # (pick_poses, _, success) = self.search_graspability(props.robot_name,
        #                                                     props.camera_name,
        #                                                     item.part_id,
        #                                                     item.bin_id,
        #                                                     props.use_normals,
        #                                                     marker_lifetime)
        self.graspability_send_goal(props.robot_name, props.camera_name,
                                    item.part_id, item.bin_id,
                                    props.use_normals)
        (pick_poses, _, success) = self.graspability_wait_for_result(
                                        props.camera_name, marker_lifetime)
        if not success:
            return False

        # Attempt to pick the item.
        nattempts = 0
        for pose in pick_poses:
            if nattempts == max_attempts:
                break

            if self._is_close_to_fail_poses(pose):
                continue

            result = self.pick(props.robot_name, pose,
                               speed_slow=props.speed_slow,
                               grasp_offset=props.grasp_offset,
                               approach_offset=props.approach_offset)
            if result == amsg.pickOrPlaceResult.SUCCESS:
                result = self.place_at_frame(
                                props.robot_name, props.destination,
                                speed_slow=props.speed_slow,
                                place_offset=props.place_offset,
                                approach_offset=props.approach_offset)
                return result == amsg.pickOrPlaceResult.SUCCESS
            elif result == amsg.pickOrPlaceResult.MOVE_FAILURE or \
                 result == amsg.pickOrPlaceResult.APPROACH_FAILURE:
                self._fail_poses.append(pose)
            elif result == amsg.pickOrPlaceResult.DEPARTURE_FAILURE:
                self.release(props.robot_name)
                raise RuntimeError("Failed to depart from pick/place pose")
            elif result == amsg.pickOrPlaceResult.PICK_FAILURE:
                self._fail_poses.append(pose)
                nattempts += 1

            self.release(props.robot_name)

        return False

    def clear_fail_poses(self):
        self._fail_poses = []

    def _is_eye_on_hand(self, robot_name, camera_name):
        return camera_name == robot_name + "_camera"

    def _is_close_to_fail_poses(self, pose):
        for fail_pose in self._fail_poses:
            if self._is_close_to_fail_pose(pose, fail_pose, 0.005):
                return True
        return False

    def _is_close_to_fail_pose(self, pose, fail_pose, tolerance):
        position      = pose.pose.position
        fail_position = fail_pose.pose.position
        if abs(position.x - fail_position.x) > tolerance or \
           abs(position.y - fail_position.y) > tolerance or \
           abs(position.z - fail_position.z) > tolerance:
            return False
        return True


if __name__ == '__main__':
    with CeatecDemoRoutines() as demo:
        while not rospy.is_shutdown():
            print("============ CeatecDemo procedures ============ ")
            print("  b: Create a backgroud image")
            print("  m: Create a mask image")
            print("  s: Search graspabilities")
            print("  a: Attempt to pick and place")
            print("  A: Repeat attempts to pick and place")
            print("  k: Do demo task")
            print("  H: Move all robots to home")
            print("  B: Move all robots to back")
            print("  q: Quit")

            try:
                key = raw_input(">> ")
                if key == 'q':
                    if demo.former_robot_name != None:
                        demo.go_to_named_pose("home",
                                                 demo.former_robot_name)
                    break
                elif key == 'H':
                    demo.go_to_named_pose("ready", "arm")
                elif key == 'B':
                    demo.go_to_named_pose("tucking", "arm")
                elif key == 'b':
                    demo.create_background_image("head_camera")
                elif key == 'm':
                    demo.create_mask_image("head_camera", demo.nbins)
                elif key == 's':
                    item  = demo.item(raw_input("  item name? "))
                    props = item.item_props
                    # demo.search_graspability(props.robot_name,
                    #                             props.camera_name,
                    #                             item.part_id, item.bin_id,
                    #                             props.use_normals,
                    #                             marker_lifetime=0)
                    demo.graspability_send_goal(props.robot_name,
                                                props.camera_name,
                                                item.part_id, item.bin_id,
                                                props.use_normals)
                    demo.graspability_wait_for_result(props.camera_name,
                                                      marker_lifetime=0)
                elif key == 'a':
                    bin = raw_input("  bin name? ")
                    demo.clear_fail_poses()
                    demo.attempt_bin(bin, 5, 0)
                    demo.go_to_named_pose("home", demo.former_robot_name)
                elif key == 'A':
                    bin = raw_input("  bin name? ")
                    demo.clear_fail_poses()
                    while demo.attempt_bin(bin, 5, 0):
                        pass
                    demo.go_to_named_pose("home", demo.former_robot_name)
                elif key == 'k':
                    demo.run()
            except Exception as e:
                print(e.message)
