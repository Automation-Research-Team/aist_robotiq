#!/usr/bin/env python
# encoding: utf-8

from math import pi

import rospy
import geometry_msgs.msg
import tf

from aist_routines.base import AISTBaseRoutines

class AISTBaseRoutinesTest(AISTBaseRoutines):
    def __init__(self):
        super(AISTBaseRoutinesTest, self).__init__()

    def robotiq_open_close_test(self, robot_name):
        rospy.loginfo('The gripper of ' + str(robot_name) + ' will open.')
        self.send_gripper_command(robot_name, 'open')
        rospy.loginfo('The gripper of ' + str(robot_name) + ' will close.')
        self.send_gripper_command(robot_name, 'close')
        rospy.loginfo('The gripper of ' + str(robot_name) + ' will open.')
        self.send_gripper_command(robot_name, 'open')

    def pick_and_place_test(self, robot_name):
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = 'workspace_center'
        object_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, pi/2, 0))
        object_pose.pose.position.z = 0.005
        rospy.loginfo('Pick up object from workspace')
        res = self.pick(robot_name, object_pose, grasp_height=0.02, speed_fast=1.0, speed_slow=0.1, gripper_command='', approach_height=0.15)
        if not res:
            rospy.loginfo('Pick was failed.')
            return

        place_pose = geometry_msgs.msg.PoseStamped()
        place_pose.header.frame_id = 'workspace_center'
        place_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(0, pi/2, 0))
        place_pose.pose.position.z = 0.005
        place_pose.pose.position.x = -.25
        rospy.loginfo('Pick up object from workspace')
        res = self.place(robot_name, place_pose, place_height=0.02, speed_fast=1.0, speed_slow=0.1, gripper_command='', approach_height=0.15)
        if not res:
            rospy.loginfo('Place was failed.')
            return

        self.go_to_named_pose('home', robot_name)

if __name__ == "__main__":
    try:
        t = AISTBaseRoutinesTest()

        while not rospy.is_shutdown():
            rospy.loginfo('='*16 + ' AISTBaseRoutine test snippets ' + '='*16)
            rospy.loginfo('11: Gripper open/close test with a_bot.')
            rospy.loginfo('12: Pick and place test with a_bot.')
            rospy.loginfo('x: Exit.')
            rospy.loginfo('')
            i = raw_input()

            if i == '11':
                t.robotiq_open_close_test('a_bot')
            elif i == '12':
                t.pick_and_place_test('a_bot')
            elif i == 'x':
                break

    except rospy.ROSInterruptException:
        pass

    print('='*8 + ' Done!!!')
