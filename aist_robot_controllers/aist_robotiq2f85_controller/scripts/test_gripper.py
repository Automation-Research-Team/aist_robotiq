#!/usr/bin/env python
import time
import rospy
import sys
import moveit_commander
import moveit_msgs.msg

import actionlib
import rospy

from moveit_python import (MoveGroupInterface)

from control_msgs.msg import GripperCommandAction, GripperCommandGoal

class GripperActionClient(object):
    def __init__(self):
        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient(
            "a_bot_gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")

    def move_gripper(self, gripper_x, max_effort, timeout=5.0):
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = max_effort
        gripper_goal.command.position = gripper_x
        self.gripper_client.send_goal(gripper_goal)
        result = self.gripper_client.wait_for_result(rospy.Duration(timeout))
        rospy.loginfo("result %d", result)
        # rospy.sleep(1.5)
        return result

class MoveFetch(object):
    def __init__(self):
        rospy.loginfo("In Move Fetch Calss init...")

        self.gripper_action = GripperActionClient()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.logdebug("moveit_commander initialised...")

        rospy.logdebug("Starting Robot Commander...")
        self.robot = moveit_commander.RobotCommander()
        rospy.logdebug("Starting Robot Commander...DONE")

        rospy.loginfo("FETCH ready to move!")

    def move_manager(self, joints_array_requested, movement_type_requested):
        success = False
        if movement_type_requested == "GRIPPER":
            gripper_x = joints_array_requested[0]
            max_effort = joints_array_requested[1]
            success = self.move_gripper(gripper_x, max_effort)
        else:
            rospy.logerr("Asked for non supported movement type==>" +
                         str(movement_type_requested))
        return success

    def move_gripper(self, gripper_x, max_effort):
        result = self.gripper_action.move_gripper(gripper_x, max_effort)
        return result

def move_tests():
    move_fetch_obj = MoveFetch()

    seq_num = 0
    raw_input("Start...Go Right")

    while not rospy.is_shutdown():
        print("Seq Num="+str(seq_num))

        if (seq_num % 2 > 0):
            print("CLOSE GRIPPER")
            max_effort = 0.1
            grip_position = 0.0
            move_fetch_obj.move_manager(
                        joints_array_requested=[ grip_position, max_effort ],
                        movement_type_requested="GRIPPER")

        if (seq_num % 2 == 0):
            print("OPEN GRIPPER")
            max_effort = 0.1
            grip_position = 0.08
            move_fetch_obj.move_manager(
                        joints_array_requested=[ grip_position, max_effort ],
                        movement_type_requested="GRIPPER")

        seq_num += 1
        if seq_num > 10:
            break
        rospy.sleep(2)

if __name__ == '__main__':
    rospy.init_node('aist_robotiq2f85_controller_node', anonymous=True, log_level=rospy.DEBUG)
    move_tests()

