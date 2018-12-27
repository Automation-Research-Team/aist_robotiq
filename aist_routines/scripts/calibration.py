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

        self.bin_names = ['bin_3_part_5']
        

        rospy.loginfo("Calibration class is staring up!")


    def touch_the_table(self):
        rospy.loginfo("Calibrating between robot and table.")
        self.go_to_named_pose("home", "b_bot")

        poses = []
        pose_b = geometry_msgs.msg.PoseStamped()
        pose_b.header.frame_id = "workspace_center"
        pose_b.pose.orientation = self.downward_orientation
        pose_b.pose.position.x = .0
        pose_b.pose.position.y = .0
        pose_b.pose.position.z = .03
        rospy.loginfo("============ Going to 3 cm above the table. ============")
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

    def bin_corner_calibration(self, robot_name="b_bot", end_effector_link=""):
        rospy.loginfo("============ Calibrating bin. ============")
        rospy.loginfo(robot_name + " end effector should be 3 cm above each corner of each bin.")
        
        if end_effector_link == "":
            self.go_to_named_pose("home", robot_name)

        poses = []

        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.position.z = 0.03
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


if __name__ == '__main__':

    try:
        c = CalibrationClass()

        while not rospy.is_shutdown():
            rospy.loginfo("============ Calibration procedures ============ ")
            rospy.loginfo("1: Go home with b_bot")
            rospy.loginfo("2: Touch the table (workspace_center)")
            rospy.loginfo("332: Bin corners with b_bot")
            rospy.loginfo("x: Exit")

            i = raw_input()
            if i == '1':
                c.go_to_named_pose("home", "b_bot")
            elif i == '2':
                c.touch_the_table()
            elif i == '332':
                c.bin_corner_calibration(robot_name="b_bot")
            if i == 'x':
                break
        print("================ done!!")
    except rospy.ROSInterruptException:
        pass
