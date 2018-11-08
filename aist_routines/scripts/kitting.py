#!/usr/bin/env python

from math import pi
import random

import rospy
import actionlib
import tf_conversions
import std_msgs.msg
import geometry_msgs.msg

from aist_routines.base import AISTBaseRoutines
import o2as_msgs.msg


class Item(object):
    """Definition class of target item"""
    def __init__(self, part_id, bin_name):
        self.part_id = part_id
        self.bin_name = bin_name


class KittingClass(AISTBaseRoutines):
    """Implements kitting routines for aist robot system."""

    def __init__(self):
        """Initialize class object."""
        super(KittingClass, self).__init__()
        self.initialize_parameters()
        self.setup_suction_tool()
        rospy.loginfo("Kitting class is staring up!")

    def initialize_parameters(self):
        """Initialize class parameters."""
        self.use_real_robot = rospy.get_param("use_real_robot", False)

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

    def setup_suction_tool(self):
        self._suction = actionlib.SimpleActionClient('o2as_fastening_tools/suction_control', SuctionControlAction)
        self._suctioned = False
        self._suction_state = rospy.Subscriber("suction_tool/screw_suctioned", std_msgs.msg.Bool, self._suction_state_callback)


    def pick(self, robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height = 0.05, special_pick = False):
        # If the pick uses suction, pass it to the local function. Otherwise to the parent class.
        if gripper_command == "suction":
            object_pose.pose.position.z += approach_height
            self.move_lin(robot_name, object_pose, speed_fast, end_effector_link="b_bot_suction_tool_tip_link")
            object_pose.pose.position.z -= approach_height
            rospy.loginfo("Try picking up by suction.")
            res = self.suck(turn_suction_on=True, eject=False, timeout=2.0)
            if not res:
                return False
            rospy.loginfo("Pushing into the bin.")
            if self.use_real_robot:
                self.do_linear_push("b_bot", force=5.0, wait=True, max_approach_distance=.092, forward_speed=.04)
                self.confirm_to_proceed("Went to the target. Press enter")
            else:
                self.move_lin(group_name, pose_goal_stamped, speed, end_effector_link=end_effector_link)
            object_pose.pose.position.z += approach_height
            self.move_lin(robot_name, object_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
            object_pose.pose.position.z -= approach_height
            if self._suctioned:
                object_pose.pose.position.z += approach_height + .05
                self.move_lin(robot_name, object_pose, speed_slow, end_effector_link="b_bot_suction_tool_tip_link")
                object_pose.pose.position.z -= approach_height + .05
            return self._suctioned
        else:
            pass
            # return super(KittingClass, self).pick(robot_name, object_pose, grasp_height, speed_fast, speed_slow, gripper_command, approach_height, special_pick)

    def get_random_pose_in_bin(self, item):
        """Get item's random pose in parts bin."""
        pick_pose = geometry_msgs.msg.PoseStamped()
        pick_pose.header.frame_id = item.bin_name

        if "bin1" in item.bin_name:
            bin_width = self.bin_1_width
            bin_length = self.bin_1_length
        elif "bin2" in item.bin_name:
            bin_width = self.bin_2_width
            bin_length = self.bin_2_length
        elif "bin3" in item.bin_name:
            bin_width = self.bin_3_width
            bin_length = self.bin_3_length

        pick_pose.pose.position.x += -bin_width/2 + random.random()*bin_width
        pick_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, pi))
        pick_pose.pose.position.y += -bin_length/2 + random.random()*bin_length

        return pick_pose

    def suck(self, turn_suction_on=False, eject=False, timeout=2.0):
        """Judge success or fail using pressure status."""
        if not self.use_real_robot:
            return True

        if turn_suction_on and eject:
            rospy.logwarn("Warning: Unexpected action might occur because suction and blow is both on.")
            return False

        goal = SuctionControlGoal()
        goal.fastening_tool_name = "suction_tool"
        goal.turn_suction_on = turn_suction_on
        goal.eject_screw = eject
        self._suction.send_goal(goal)
        self._suction.wait_for_result(rospy.Duration(timeout))
        return self._suction.get_result()

    def _suction_state_callback(self, msg):
        self._suctioned = msg.data

if __name__ == '__main__':

    rospy.init_node("Kitting")

    try:
        kit = KittingClass()

        while not rospy.is_shutdown():
            rospy.loginfo("x: Exit")

            i = raw_input()
            if i == '1':
                pass
            elif i == 'x':
                break
        print("================ Done!")
    except rospy.ROSInterruptException:
        pass
