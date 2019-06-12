#!/usr/bin/env python

from math import pi
import copy

import rospy
import tf
import tf_conversions
import geometry_msgs.msg

from aist_routines.base import AISTBaseRoutines

######################################################################
#  global variables                                                  #
######################################################################
orientations = {
    'o2as': {
        'a_bot': [radians(-90), radians( 90), radians(0)],
        'b_bot': [radians(  0), radians( 90), radians(0)],
        'c_bot': [radians(  0), radians( 90), radians(0)],
    },

    'aist': {
        'a_bot': [radians(-90), radians( 90), radians(0)],
        'b_bot': [radians(  0), radians( 90), radians(0)],
    },

    'pgrp': {
        'a_bot': [radians(-90), radians( 90), radians(0)],
        'b_bot': [radians(  0), radians( 90), radians(0)],
    },

    'ur5e': {
        'c_bot': [radians(-90), radians( 90), radians(0)],
        'd_bot': [radians(  0), radians( 90), radians(0)],
    },
}

######################################################################
#  class BinCalibrationRoutines                                      #
######################################################################
class BinCalibrationRoutines(AISTBaseRoutines):
    def __init__(self, robot_name):
        super(BinCalibrationRoutines, self).__init__(orientation)
        self._bins = {
            "bin_2" : "part_4",
            "bin_2" : "part_7",
            "bin_2" : "part_8",
            "bin_3" : "part_15",
            "bin_3" : "part_16",
        }
        self.robot_name   = robot_name
        rospy.loginfo("Calibration class is staring up!")

    @property
    def robot_name(self):
        return self.group.get_name()

    @robot_name.setter
    def robot_name(self, name):
        self._group = moveit_commander.MoveGroupCommander(name)
        self._group.set_pose_reference_frame("workspace_center")
        self._group.set_end_effector_link(name + "_ee_link")

    def move(self, pose)
        poseStamped = gmsg.PoseStamped()
        poseStamped.header.frame_id = self.group.get_pose_reference_frame()
        poseStamped.pose = gmsg.Pose(
            gmsg.Point(pose[0], pose[1], pose[2]),
            gmsg.Quaternion(
                *tfs.quaternion_from_euler(pose[3], pose[4], pose[5])))
        print("  move to " + self.format_pose(poseStamped))
        (success, _, current_pose) = self.go_to_pose_goal(
                self.robot_name, poseStamped, self.speed,
                end_effector_link=self.group.get_end_effector_link(),
                move_lin=True)
        print("  reached " + self.format_pose(current_pose))
        return success

    def touch_the_table(self):
        rospy.loginfo("Calibrating between robot and table.")
        self.go_to_named_pose("home", self.robot_name)

        poses = []
        pose_b = geometry_msgs.msg.PoseStamped()
        pose_b.header.frame_id = self.group.get_pose_reference_frame()
        if robot_name == 'a_bot':
            pose_b.pose.orientation = self.downward_orientation_a_bot
        elif robot_name == 'b_bot':
            pose_b.pose.orientation = self.downward_orientation
        pose_b.pose.position.x = .0
        pose_b.pose.position.y = .0
        pose_b.pose.position.z = .03
        rospy.loginfo("============ Going to 3 cm above the table. ============")
        self.go_to_pose_goal(robot_name, pose_b, speed=0.5, acceleration=self.acceleration, high_precision=False, end_effector_link="", move_lin=True)

        rospy.loginfo("============ Press enter to go to 1 cm above the table. ============")
        i = raw_input()
        if not rospy.is_shutdown():
            pose_b.pose.position.z = .01
            self.go_to_pose_goal(robot_name, pose_b, speed=0.01, acceleration=self.acceleration, high_precision=False, end_effector_link="", move_lin=True)

        rospy.loginfo("============ Press enter to go home. ============")
        raw_input()
        self.go_to_named_pose("home", robot_name)
        return


    def bin_calibration(self, robot_name="b_bot", end_effector_link=""):
        rospy.loginfo("============ Calibrating bins. ============")
        rospy.loginfo(robot_name + " end effector should be 3 cm above center of bin.")

        if end_effector_link=="":
            self.go_to_named_pose("home", robot_name)

        poses = []

        pose0 = geometry_msgs.msg.PoseStamped()
        if robot_name == 'a_bot':
            pose0.pose.orientation = self.downward_orientation_a_bot
        elif robot_name == 'b_bot':
            pose0.pose.orientation = self.downward_orientation
        pose0.pose.position.z = 0.03

        for bin in self.bin_names:
            pose0.header.frame_id = bin
            world_pose = self.listener.transformPose("workspace_center", pose0)
            poses.append(copy.deepcopy(pose0))

        self.cycle_through_calibration_poses(poses, robot_name, speed=0.1, end_effector_link=end_effector_link, move_lin=True, go_home=False)
        return


    def bin_corner_calibration(self, robot_name="b_bot", end_effector_link=""):
        rospy.loginfo("============ Calibrating bin. ============")
        rospy.loginfo(robot_name + " end effector should be 3 cm above each corner of each bin.")

        if end_effector_link == "":
            # FIXME: What does it mean? I think it is a bug.
            self.go_to_named_pose("home", robot_name)

        poses = []

        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.position.z = 0.03
        if robot_name == 'a_bot':
            pose0.pose.orientation = self.downward_orientation_a_bot
        elif robot_name == 'b_bot':
            pose0.pose.orientation = self.downward_orientation

        for bin in self.bin_names:
            pose0.header.frame_id = bin
            world_pose = self.listener.transformPose("workspace_center", pose0)
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


    def workspace_calibration(self, robot_name="b_bot", end_effector_link=""):
        rospy.loginfo("================ Calibrating workspace. ================")
        rospy.loginfo(robot_name + " end effector should be 3 cm above the table.")

        if end_effector_link == "":
            if robot_name == "a_bot":
                end_effector_link = "a_bot_robotiq_85_tip_link"
            elif robot_name == "b_bot":
                end_effector_link = "b_bot_single_suction_effector_pad_link"

        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.header.frame_id = "workspace_center"
        pose0.pose.position.y = 0.2
        pose0.pose.position.z = 0.01
        if robot_name == 'a_bot':
            pose0.pose.orientation = self.downward_orientation_a_bot
        elif robot_name == 'b_bot':
            pose0.pose.orientation = self.downward_orientation
        poses = []
        for i in xrange(7):
            poses.append(copy.deepcopy(pose0))
        poses[0].pose.position.x = 0.00
        poses[1].pose.position.x = -0.10
        poses[2].pose.position.x = -0.20
        poses[3].pose.position.x = -0.25
        poses[4].pose.position.x = 0.10
        poses[5].pose.position.x = 0.20
        poses[6].pose.position.x = 0.25

        c.cycle_through_calibration_poses(poses, robot_name, speed=.05, end_effector_link=end_effector_link, move_lin=True, go_home=False)
        return

    def workspace_calibration_interactive(self, robot_name="b_bot", end_effector_link=""):
        rospy.loginfo("================ Calibrating workspace. ================")
        rospy.loginfo(robot_name + " end effector should be 3 cm above the table.")

        if end_effector_link == "":
            if robot_name == "a_bot":
                end_effector_link = "a_bot_robotiq_85_tip_link"
            elif robot_name == "b_bot":
                end_effector_link = "b_bot_single_suction_gripper_pad_link"

        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "workspace_center"
        if robot_name == 'a_bot':
            pose.pose.orientation = self.downward_orientation_a_bot
        elif robot_name == 'b_bot':
            pose.pose.orientation = self.downward_orientation
        pose.pose.position.x = float(raw_input("x >> "))
        pose.pose.position.y = float(raw_input("y >> "))
        pose.pose.position.z = float(raw_input("z >> "))
        self.go_to_pose_goal(robot_name, pose, speed=0.5, acceleration=self.acceleration, high_precision=False, end_effector_link=end_effector_link, move_lin=True)

        rospy.loginfo("Target position: " + str(pose))
        current_pose_in_world = self.groups[robot_name].get_current_pose()
        current_pose_in_workspace = self.listener.transformPose('workspace_center', current_pose_in_world)
        rospy.loginfo("Current position: " + str())
        rospy.loginfo(current_pose_in_workspace)

        return


if __name__ == '__main__':

    try:
        c = CalibrationClass()

        while not rospy.is_shutdown():
            rospy.loginfo("============ Calibration procedures ============ ")
            rospy.loginfo("1: Go home with all_bot (not implemented)")
            rospy.loginfo("11, 12: Go home with a_bot, b_bot")
            rospy.loginfo("211, 212: Touch the table (workspace_center) with a_bot, b_bot")
            rospy.loginfo("221, 222: Workspace calibration with a_bot, b_bot")
            rospy.loginfo("231, 232: Interactive workspace calibration with b_bot")
            rospy.loginfo("322: Bins with b_bot")
            rospy.loginfo("332: Bin corners with b_bot")
            rospy.loginfo("x: Exit")

            i = raw_input()
            if i == '11':
                c.go_to_named_pose("home", "a_bot")
            elif i == '12':
                c.go_to_named_pose("home", "b_bot")
            elif i == '211':
                c.touch_the_table('a_bot')
            elif i == '212':
                c.touch_the_table('b_bot')
            elif i == '221':
                c.workspace_calibration("a_bot")
            elif i == '222':
                c.workspace_calibration("b_bot")
            elif i == '231':
                c.workspace_calibration_interactive("a_bot")
            elif i == '232':
                c.workspace_calibration_interactive("b_bot")
            elif i == '322':
                c.bin_calibration(robot_name="b_bot")
            elif i == '332':
                c.bin_corner_calibration(robot_name="b_bot")
            if i == 'x':
                break
        print("================ done!!")
    except rospy.ROSInterruptException:
        pass
