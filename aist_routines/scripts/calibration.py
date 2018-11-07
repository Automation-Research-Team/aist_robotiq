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

        rospy.loginfo("Calibration class is staring up!")

    def cycle_through_calibration_poses(self, poses, robot_name, speed=0.3, move_lin=False, go_home=True, end_effector_link=""):
        home_pose = "home"
        if "screw" in end_effector_link:
            home_pose = "screw_ready"

        for pose in poses:
            rospy.loginfo("============ Press `Enter` to move " + robot_name + " to " + pose.header.frame_id)
            # self.publish_marker(pose, "place_pose")
            raw_input()
            if go_home:
                self.go_to_named_pose(home_pose, robot_name)
            if rospy.is_shutdown():
                break
            else:
                self.go_to_pose_goal(robot_name, pose,speed=speed,end_effector_link=end_effector_link, move_lin = move_lin)

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
        pose_b.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
        pose_b.pose.position.x = .0
        pose_b.pose.position.y = .0
        pose_b.pose.position.z = .03
        rospy.loginfo("============ Going to 2 cm above the table. ============")
        self.go_to_pose_goal("b_bot", pose_b, speed=1.0, acceleration=0.0, high_precision=False, end_effector_link="", move_lin=True)

        rospy.loginfo("============ Press enter to go to .1 cm above the table. ============")
        i = raw_input()
        if not rospy.is_shutdown():
            pose_b.pose.position.z = .001
            self.go_to_pose_goal("b_bot", pose_b, speed=0.01, acceleration=0.0, high_precision=False, end_effector_link="", move_lin=True)

        rospy.loginfo("============ Press enter to go home. ============")
        raw_input()
        self.go_to_named_pose("home", "b_bot")
        return

    def check_calibration_tray_position(self, robot_name, tray_name):
        rospy.loginfo("Calibrating tray_position")

        corners = [
            "top_front_left_corner",
            "top_back_left_corner",
            "top_front_right_corner",
            "top_back_right_corner"
        ]

        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
        pose0.pose.position.z = 0.01
        poses = []
        for corner in corners:
            pose = copy.deepcopy(pose0)
            pose.header.frame_id = tray_name + "_" + corner
            poses.append(pose)

        self.cycle_through_calibration_poses(poses, robot_name, speed = 1.0, move_lin=True, go_home=False, end_effector_link="b_bot_suction_tool_tip_link")

if __name__ == '__main__':

    rospy.init_node("Calibration")

    try:
        c = CalibrationClass()

        while True:
            rospy.loginfo("================ MoveIt examples ================")
            rospy.loginfo("Enter 1 to move b_bot to home pose.")
            rospy.loginfo("Enter 2 to touch the table (workspace_center).")
            rospy.loginfo("Enter 31(311, 312) to calibrate tray position(set_1_tray_1/set_1_tray_2).")
            rospy.loginfo("Enter x to exit.")

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
            if i == 'x':
                break
        print("================ done!!")
    except rospy.ROSInterruptException:
        pass
