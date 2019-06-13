#!/usr/bin/env python

import os, csv, copy, time, datetime, re

import rospy
import rospkg
import tf_conversions
import std_msgs.msg
import geometry_msgs.msg
import actionlib

from aist_routines.base import AISTBaseRoutines
import o2as_msgs.msg

######################################################################
#  global variables                                                  #
######################################################################
rp = rospkg.RosPack()

ts = time.time()
start_date_time = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H-%M-%S')
number_of_attempted = 1

######################################################################
#  global functions                                                  #
######################################################################
def clamp(n, minn, maxn):
    """Constrain a number n to the interval [minn, maxn]"""
    return min(max(n, minn), maxn)

######################################################################
#  class PartProperties                                              #
######################################################################
class PartProperty():
    def __init__(self, id, robot_name, camera_name, target_frame,
                 dropoff_height):
        self._id             = id
        self._robot_name     = robot_name
        self._camera_name    = camera_name
        self._target_frame   = target_frame
        self._dropoff_height = dropoff_height

    @property
    def id(self):
        return self._id

    @property
    def robot_name(self):
        return self._robot_name

    @property
    def camera_name(self):
        return self._camera_name

    @property
    def target_frame(self):
        return self._target_frame

    @property
    def dropoff_height(self):
        return self._dropoff_height


######################################################################
#  class KittingRoutines                                             #
######################################################################
class KittingRoutines(AISTBaseRoutines):
    """Implements kitting routines for aist robot system."""
    _part_properties = {
        "part_4"  : PartProperty( 4, "b_bot", "a_phoxi_m_camera",
                                  "tray_1_partition_4", 0.15),
        "part_5"  : PartProperty( 5, "b_bot", "a_phoxi_m_camera",
                                  "tray_2_partition_6", 0.15),
        "part_6"  : PartProperty( 6, "b_bot", "a_phoxi_m_camera",
                                  "tray_1_partition_3", 0.15),
        "part_7"  : PartProperty( 7, "b_bot", "a_phoxi_m_camera",
                                  "tray_1_partition_2", 0.15),
        "part_8"  : PartProperty( 8, "b_bot", "a_phoxi_m_camera",
                                  "tray_2_partition_1", 0.15),
        "part_9"  : PartProperty( 9, "a_bot", "a_phoxi_m_camera",
                                  "tray_2_partition_4", 0.15),
        "part_10" : PartProperty(10, "a_bot", "a_phoxi_m_camera",
                                 "tray_2_partition_7", 0.15),
        "part_11" : PartProperty(11, "b_bot", "a_phoxi_m_camera",
                                 "tray_1_partition_1", 0.15),
        "part_12" : PartProperty(12, "b_bot", "a_phoxi_m_camera",
                                 "tray_2_partition_3", 0.15),
        "part_13" : PartProperty(13, "b_bot", "a_phoxi_m_camera",
                                 "tray_1_partition_5", 0.15),
        "part_14" : PartProperty(14, "a_bot", "a_phoxi_m_camera",
                                 "tray_2_partition_2", 0.15),
        "part_15" : PartProperty(15, "a_bot", "a_phoxi_m_camera",
                                 "tray_2_partition_5", 0.15),
        "part_16" : PartProperty(16, "a_bot", "a_phoxi_m_camera",
                                 "tray_2_partition_8", 0.15),
        "part_17" : PartProperty(17, "a_bot", "a_phoxi_m_camera",
                                 "skrewholder_1",      0.15),
        "part_18" : PartProperty(18, "a_bot", "a_phoxi_m_camera",
                                 "skrewholder_2",      0.15),
    }

    def __init__(self):
        super(KittingClass, self).__init__()
        self._bins = [
            "bin_2_part_4",
            "bin_2_part_7",
            "bin_2_part_8",
            "bin_3_part_15",
            "bin_3_part_16",
        ]
        self._items = {}
        for bin in self._bins:
            part_name = re.search("part_[0-9]*", bin).group()
            self._items[bin] = KittingRoutines._part_properties[part_name]
        self.go_to_named_pose("home", "all_bots")

    @property
    def robot_name(self):
        return self._group.get_name()

    @robot_name.setter
    def robot_name(self, name):
        self._group = moveit_commander.MoveGroupCommander(name)
        self._group.set_pose_reference_frame("workspace_center")
        if self.gripper(name).type == "suction":
            self.gripper(name).release()
        else:
            self.gripper(name).grasp()

    ###----- main procedure
    def kitting_task(self):
        for bin in self._bins:
            if rospy.is_shutdown():
                break
            self.attempt_bin(bin, 1)
        self.go_to_named_pose("home", "all_bots")

    def attempt_bin(self, bin, max_attempts=5):
        attempts = 0
        (poses, rotipz, gscore, success) \
            = self.search_graspability(item.robot_name, item.camera_name,
                                       item.part_id)

        item = self._items[bin]

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
                pick_number = random.randint(0, len(self.grasp_candidates[item.part_id]["position"]) - 1)
                pick_pose = self.grasp_candidates[item.part_id]["position"].pop(pick_number)
                # pick_pose = self.grasp_candidates[item.part_id]["position"].pop(0)
                rospy.loginfo("Pick pose in bin:")
                rospy.loginfo(pick_pose)
            pick_pose.pose.orientation = self.downward_orientation
            approach_height = 0.15
            if item.ee_to_use == "suction":
                gripper_command = "suction"
                approach_height = .15
                pick_pose.pose.position.z -= .003
            else:
                gripper_command = ""
            # pick_pose = self.make_pose_safe_for_bin(pick_pose, item)
            self.publish_marker(pick_pose, "aist_vision_result")
            item_picked = self.pick(robot_name, pick_pose, grasp_height=0.0, speed_fast=.1, speed_slow=.05,
                                                gripper_command=gripper_command, approach_height=approach_height, timeout=10.0)
            if not self.use_real_robot:
                item_picked = True
            if not item_picked:
                rospy.logerr("Failed an attempt to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + "). Reattempting. Current attempts: " + str(attempts))
                if item.ee_to_use == "suction":
                    self.suck(turn_suction_on=False, eject=False)
                continue

            # Attempt to place the item
            place_pose = geometry_msgs.msg.PoseStamped()
            if self.is_aist_experiment:
                place_pose.header.frame_id = "place_bin"
            else:
                place_pose.header.frame_id = item.target_frame
            place_pose.pose.orientation = self.downward_orientation2
            approach_height = .15
            if item.ee_to_use == "suction":
                self.go_to_named_pose("intermediate_pose_to_place_item", "b_bot", speed=1.0, acceleration=1.0)
            self.place(robot_name, place_pose, place_height=item.dropoff_height,
                                        speed_fast=1.0, speed_slow=0.05, gripper_command=gripper_command, approach_height=approach_height)
            # If successful
            self.fulfilled_items += 1
            item.fulfilled = True
            rospy.loginfo("Delivered item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")! Total items delivered: " + str(self.fulfilled_items))
            self.grasp_candidates[item.part_id]["position"] = []
            return True

        rospy.logerr("Was not able to pick item nr." + str(item.number_in_set) + " from set " + str(item.set_number) + " (part ID:" + str(item.part_id) + ")! Total attempts: " + str(item.attempts))
        if item.ee_to_use == "suction":
            self.go_to_named_pose("home", "b_bot")

        self.grasp_candidates[item.part_id]["position"] = []
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
            self._grasp_candidates_in_camera_pub.publish(obj_pose_in_camera)
            pose_in_bin = self.listener.transformPose(item.bin_name, obj_pose_in_camera)
            pose_in_bin.pose.orientation = self.downward_orientation
            if item.ee_to_use in ["precision_gripper_from_outside", "robotiq_gripper"]:
                pose_in_bin.pose.orientation = geometry_msgs.msg.Quaternion(
                    *tf_conversions.transformations.quaternion_from_euler(0, pi/2, resp_search_grasp.rotipz[i]))
            poses_in_bin.append(pose_in_bin)
            self._grasp_candidates_in_bin_pub.publish(pose_in_bin)
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


if __name__ == '__main__':


    try:
        kit = KittingClass(vision_algo="fge")

        while True:
            rospy.loginfo("")
            rospy.loginfo("1: to go to home all robots.")
            rospy.loginfo("322: Bins with b_bot")
            rospy.loginfo("332: Bin corners with b_bot")
            rospy.loginfo("Enter 71, 72... to test phoxi on item 1, 2...")
            rospy.loginfo("Enter START to start the task.")
            rospy.loginfo("Enter x to exit.")

            i = raw_input()
            if i == '1':
                kit.go_to_named_pose("home", "b_bot")
            elif i == '322':
                kit.bin_calibration(robot_name="b_bot")
            elif i == '332':
                kit.bin_corner_calibration(robot_name="b_bot")
            elif i in ["71", "72", "73", "74", "75", "76", "77", "78", "79"]:
                item = kit.ordered_items[int(i)-71]
                rospy.loginfo("Checking for item id " + str(item.part_id) + " in " + item.bin_name)
                kit.get_grasp_candidates_from_phoxi(item, True)
                rospy.loginfo("Grasp candidates of item " + str(item.part_id))
                rospy.loginfo(kit.grasp_candidates[item.part_id]["position"])
                try:
                    kit.publish_marker(kit.grasp_candidates[item.part_id]["position"].pop(0), "aist_vision_result")
                except IndexError:
                    pass
            elif i == 'START' or i == 'start':
                kit.kitting_task()
            elif i == 'x':
                break
        print("================ Done!")
    except rospy.ROSInterruptException:
        pass
