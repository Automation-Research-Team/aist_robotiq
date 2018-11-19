#!/usr/bin/env python

from math import pi
import copy

import rospy
import tf
import tf_conversions
import geometry_msgs.msg

from aist_routines.base import AISTBaseRoutines

class CalibrationClass(AISTBaseRoutines):
    def __init__(self):
        super(CalibrationClass, self).__init__()
        self.use_real_robot = rospy.get_param("use_real_robot", False)
        if self.use_real_robot:
            self.acceleration = 1.0
        else:
            self.acceleration = 0.0

        self.bin_names = [
            "bin_2_part_11",
            "bin_2_part_7",
            "bin_2_part_4",
            "bin_2_part_8",
            "bin_2_part_5",
            "bin_1_part_17",
            "bin_1_part_13",
            "bin_1_part_18",
            "bin_1_part_15",
            "bin_1_part_10"
        ]

        rospy.loginfo("Calibration class is staring up!")

    def cycle_through_calibration_poses(self, poses, robot_name, speed=0.3, move_lin=False, go_home=True, end_effector_link=""):
        home_pose = "home"
        if "screw" in end_effector_link:
            home_pose = "screw_ready"

        for pose in poses:
            rospy.loginfo("============ Press `Enter` to move " + robot_name + " to " + pose.header.frame_id)
            self.publish_marker(pose, "place_pose")
            raw_input()
            if go_home:
                self.go_to_named_pose(home_pose, robot_name)
            if rospy.is_shutdown():
                break
            else:
                self.go_to_pose_goal(robot_name, pose,speed=speed, acceleration=self.acceleration, end_effector_link=end_effector_link, move_lin = move_lin)

            rospy.loginfo("============ Press `Enter` to proceed ")
            raw_input()

            if go_home:
                self.go_to_named_pose(home_pose, robot_name, force_ur_script=move_lin)

        if go_home:
            rospy.loginfo("Moving all robots home again.")
            self.go_to_named_pose("home", "b_bot")
        return

    def touch_the_table(self):
        rospy.loginfo("Calibrating between robot and table.")
        self.go_to_named_pose("home", "b_bot")

        poses = []
        pose_b = geometry_msgs.msg.PoseStamped()
        pose_b.header.frame_id = "workspace_center"
        pose_b.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pose_b.pose.position.x = .0
        pose_b.pose.position.y = .0
        pose_b.pose.position.z = .03
        rospy.loginfo("============ Going to 2 cm above the table. ============")
        self.go_to_pose_goal("b_bot", pose_b, speed=0.5, acceleration=self.acceleration, high_precision=False, end_effector_link="", move_lin=True)

        rospy.loginfo("============ Press enter to go to 1 cm above the table. ============")
        i = raw_input()
        if not rospy.is_shutdown():
            pose_b.pose.position.z = .01
            self.go_to_pose_goal("b_bot", pose_b, speed=0.01, acceleration=self.acceleration, high_precision=False, end_effector_link="", move_lin=True)

        rospy.loginfo("============ Press enter to go home. ============")
        raw_input()
        self.go_to_named_pose("home", "b_bot")
        return

    def check_calibration_tray_position(self, robot_name, tray_name):
        rospy.loginfo("Calibrating tray_position")

        corners = [
            "top_front_left_corner",
            "top_back_left_corner",
            "top_back_right_corner",
            "top_front_right_corner"
        ]

        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pose0.pose.position.z = 0.01
        poses = []
        for corner in corners:
            pose = copy.deepcopy(pose0)
            pose.header.frame_id = tray_name + "_" + corner
            poses.append(pose)

        self.cycle_through_calibration_poses(poses, robot_name, speed=0.05, move_lin=True, go_home=False, end_effector_link="b_bot_dual_suction_gripper_pad_link")

    def check_calibration_bin_rack(self, robot_name, rack_name):
        rospy.loginfo("Calibrating bin rack position")

        corners = [
            "top_front_left_corner",
            "top_back_left_corner",
            "top_back_right_corner",
            "top_front_right_corner"
        ]

        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pose0.pose.position.z = 0.01
        poses = []
        for corner in corners:
            pose = copy.deepcopy(pose0)
            pose.header.frame_id = rack_name + corner
            poses.append(pose)

        self.cycle_through_calibration_poses(poses, robot_name, speed = 0.1, move_lin=True, go_home=False, end_effector_link="b_bot_dual_suction_gripper_pad_link")

    def bin_corner_calibration(self, robot_name="b_bot", end_effector_link=""):
        rospy.loginfo("============ Calibrating bin. ============")
        rospy.loginfo(robot_name + " end effector should be 3 cm above each corner of each bin.")

        if end_effector_link=="":
            self.go_to_named_pose("home", robot_name)

        poses = []

        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pose0.pose.position.z = 0.03

        for bin in self.bin_names:
            pose0.header.frame_id = bin
            world_pose = self.listener.transformPose("workspace_center", pose0)
            # if robot_name == "b_bot":
            #     if world_pose.pose.position.y < -.15:
            #         continue
            new_pose = copy.deepcopy(pose0)
            new_pose.header.frame_id = bin + "_top_back_left_corner"
            poses.append(new_pose)
            new_pose = copy.deepcopy(pose0)
            new_pose.header.frame_id = bin + "_top_back_right_corner"
            poses.append(new_pose)
            new_pose = copy.deepcopy(pose0)
            new_pose.header.frame_id = bin + "_top_front_right_corner"
            poses.append(new_pose)
            new_pose = copy.deepcopy(pose0)
            new_pose.header.frame_id = bin + "_top_front_left_corner"
            poses.append(new_pose)

        self.cycle_through_calibration_poses(poses, robot_name, speed=0.1, end_effector_link=end_effector_link, move_lin=True, go_home=False)
        return

if __name__ == '__main__':

    rospy.init_node("Calibration")

    try:
        c = CalibrationClass()

        while not rospy.is_shutdown():
            rospy.loginfo("============ Calibration procedures ============ ")
            rospy.loginfo("1: Go home with b_bot")
            rospy.loginfo("2: Touch the table (workspace_center)")
            rospy.loginfo("31: Go to tray corners")
            rospy.loginfo("311: Go to set_1_tray_1 corners")
            rospy.loginfo("322: Go to set_1_tray_2 corners")
            rospy.loginfo("39: Go to place_bin corners")
            rospy.loginfo("411: Bin-rack calibration")
            rospy.loginfo("412: Tray-rack calibration")
            rospy.loginfo("5: Bin corners with b_bot")
            rospy.loginfo("x: Exit")

            i = raw_input()
            if i == '1':
                c.go_to_named_pose("home", "b_bot")
            elif i == '2':
                c.touch_the_table()
            elif '31' in i:
                if i == '311':
                    c.check_calibration_tray_position("b_bot", "set_1_tray_1")
                elif i == '312':
                    c.check_calibration_tray_position("b_bot", "set_1_tray_2")
                else:
                    c.check_calibration_tray_position("b_bot", "set_1_tray_1")
                    c.check_calibration_tray_position("b_bot", "set_1_tray_2")
            elif i == '39':
                c.check_calibration_tray_position("b_bot", "place_bin")
            elif i == '411':
                c.check_calibration_bin_rack("b_bot", "bin_rack")
            elif i == '412':
                c.check_calibration_bin_rack("b_bot", "tray_rack")
            elif i == '5':
                c.bin_corner_calibration("b_bot")
            if i == 'x':
                break
        print("================ done!!")
    except rospy.ROSInterruptException:
        pass
