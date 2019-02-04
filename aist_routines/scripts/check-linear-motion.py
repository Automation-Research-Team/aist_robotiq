#!/usr/bin/env python

import copy

import rospy
import geometry_msgs.msg

from aist_routines.base import AISTBaseRoutines

class LinearMotionCheckClass(AISTBaseRoutines):
    def __init__(self):
        super(LinearMotionCheckClass, self).__init__()

    def round_trip_on_axis(self, robot_name, axis='x'):
        pose0 = geometry_msgs.msg.PoseStamped()
        pose0.header.frame_id = 'workspace_center'
        pose0.pose.orientation = self.downward_orientation
        poses = []

        for i in range(0, 9):
            if axis in ['x', 'y']:
                pose0.pose.position.z = 0.01
            poses.append(copy.deepcopy(pose0))

        if axis == 'x':
            rospy.loginfo(robot_name + 'do round trip on x-axis at intervals of 0.1 meter.')
            poses[0].pose.position.x = .0
            poses[1].pose.position.x = -.1
            poses[2].pose.position.x = -.2
            poses[3].pose.position.x = -.1
            poses[4].pose.position.x = .0
            poses[5].pose.position.x = .1
            poses[6].pose.position.x = .2
            poses[7].pose.position.x = .1
            poses[8].pose.position.x = .0
        elif axis == 'y':
            rospy.loginfo(robot_name + 'do round trip on y-axis at intervals of 0.1 meter.')
            poses[0].pose.position.y = .0
            poses[1].pose.position.y = -.1
            poses[2].pose.position.y = -.2
            poses[3].pose.position.y = -.1
            poses[4].pose.position.y = .0
            poses[5].pose.position.y = .1
            poses[6].pose.position.y = .2
            poses[7].pose.position.y = .1
            poses[8].pose.position.y = .0
        elif axis == 'z':
            rospy.loginfo(robot_name + 'do round trip on x-axis at intervals of 0.05 meter.')
            poses[0].pose.position.z = .005
            poses[1].pose.position.z = .05
            poses[2].pose.position.z = .10
            poses[3].pose.position.z = .15
            poses[4].pose.position.z = .20
            poses[5].pose.position.z = .15
            poses[6].pose.position.z = .10
            poses[7].pose.position.z = .05
            poses[8].pose.position.z = .005

        self.cycle_through_calibration_poses(poses, robot_name, speed=1.0, move_lin=True, go_home=False)
        return

if __name__ == '__main__':
    try:
        m = LinearMotionCheckClass()

        while not rospy.is_shutdown():
            rospy.loginfo('=' * 16)
            rospy.loginfo('121, 122, 123: [b_bot] does round trip on [x, y, z] axis above workspace.')
            rospy.loginfo('x: Exit')
            i = raw_input()
            if i == '121':
                m.round_trip_on_axis('b_bot', 'x')
            elif i == '122':
                m.round_trip_on_axis('b_bot', 'y')
            elif i == '123':
                m.round_trip_on_axis('b_bot', 'z')
            elif i == 'x':
                break
    except rospy.ROSInterruptException:
        pass

    print('='*16 + 'done!')
