#!/usr/bin/env python

import sys
import os
import copy
import rospy
import argparse
from geometry_msgs import msg as gmsg

from tf import transformations as tfs
from math import radians, degrees
from aist_routines.iiwa import IiwaRoutines

######################################################################
#  global functions                                                  #
######################################################################
def is_num(s):
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True

######################################################################
#  class InteractiveRoutines                                         #
######################################################################
class InteractiveRoutines(IiwaRoutines):
    refposes = {
        # 'a_iiwa': [0.70, -0.20, 1.50, radians(0), radians(0), radians(0)],
        'a_iiwa': [0.30,  0.00, 2.00, radians(0), radians(0), radians(0)],
        'b_iiwa': [0.70,  1.40, 1.50, radians(0), radians(0), radians(0)],
    }

    def __init__(self, robot_name, camera_name, speed, ns):
        super(InteractiveRoutines, self).__init__(ns)

        self._robot_name   = robot_name
        self._camera_name  = camera_name
        self._speed        = speed

    def go_standing(self):
        self.go_to_named_pose('standing', self._robot_name)

    def go_home(self):
        self.go_to_named_pose('home', self._robot_name)

    """
    def go_back(self):
        self.go_to_named_pose('back', self._robot_name)
    """

    def move(self, pose):
        target_pose = gmsg.PoseStamped()
        # target_pose.header.frame_id = "workspace_center"
        target_pose.header.frame_id = "world"
        target_pose.pose = gmsg.Pose(
            gmsg.Point(pose[0], pose[1], pose[2]),
            gmsg.Quaternion(
                *tfs.quaternion_from_euler(pose[3], pose[4], pose[5])))
        (success, _, current_pose) = self.go_to_pose_goal(
                                                self._robot_name, target_pose,
                                                self._speed,
                                                move_lin=False)
        return success

    def run(self):
        # Reset pose
        # self.go_home()
        self.go_standing()

        axis = 'Y'

        _pose = None

        while not rospy.is_shutdown():
            current_pose = self.get_current_pose(self._robot_name)
            print "# current_pose #\n", current_pose
            prompt = "{:>5}:{}{:>9}>> " \
                   .format(axis, self.format_pose(current_pose), "moveit")

            key = raw_input(prompt)

            if key == 'q':
                break
            elif key == 'X':
                axis = 'X'
            elif key == 'Y':
                axis = 'Y'
            elif key == 'Z':
                axis = 'Z'
            elif key == 'R':
                axis = 'Roll'
            elif key == 'P':
                axis = 'Pitch'
            elif key == 'W':
                axis = 'Yaw'
            elif key == '+':
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] += 0.01
                elif axis == 'Y':
                    goal_pose[1] += 0.01
                elif axis == 'Z':
                    goal_pose[2] += 0.01
                elif axis == 'Roll':
                    goal_pose[3] += radians(10)
                elif axis == 'Pitch':
                    goal_pose[4] += radians(10)
                else:
                    goal_pose[5] += radians(10)
                self.move(goal_pose)
            elif key == '-':
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] -= 0.01
                elif axis == 'Y':
                    goal_pose[1] -= 0.01
                elif axis == 'Z':
                    goal_pose[2] -= 0.01
                elif axis == 'Roll':
                    goal_pose[3] -= radians(10)
                elif axis == 'Pitch':
                    goal_pose[4] -= radians(10)
                else:
                    goal_pose[5] -= radians(10)
                self.move(goal_pose)
            elif is_num(key):
                if _pose is None:
                    _pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    _pose[0] = float(key)
                elif axis == 'Y':
                    _pose[1] = float(key)
                elif axis == 'Z':
                    _pose[2] = float(key)
                elif axis == 'Roll':
                    _pose[3] = radians(float(key))
                elif axis == 'Pitch':
                    _pose[4] = radians(float(key))
                elif axis == 'Yaw':
                    _pose[5] = radians(float(key))
                print "# _pose #\n", _pose
            elif key == 'reset':
                _pose = None
            elif key == 'go':
                self.move(_pose)
            elif key == 's':
                self.stop(self._robot_name)
            elif key == 'pregrasp':
                self.pregrasp(self._robot_name)
            elif key == 'grasp':
                self.grasp(self._robot_name)
            elif key == 'release':
                self.release(self._robot_name)
            elif key == 'cont':
                self.continuous_shot(self._camera_name, True)
            elif key == 'stopcont':
                self.continuous_shot(self._camera_name, False)
            elif key == 'trigger':
                self.trigger_frame(self._camera_name)
            elif key == 'mask':
                self.create_mask_image(self._camera_name,
                                       int(raw_input("  #bins? ")))
            elif key == 'search':
                self.delete_all_markers()
                self.graspability_send_goal(self._robot_name,
                                            self._camera_name, 4, 0, True)
                (poses, gscore, success) = \
                    self.graspability_wait_for_result(self._camera_name, 0)
                for gs in gscore:
                    print(str(gs))
                print(str(poses))
            elif key == 'o':
                self.move(InteractiveRoutines.refposes[self._robot_name])
            elif key == 'standing':
                self.go_standing()
            elif key == 'h':
                self.go_home()
            """
            elif key == 'b':
                self.go_back()
            """

        # Reset pose
        self.go_home()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Perform tool calibration')
    parser.add_argument('-r',
                        '--robot_name',
                        action='store',
                        nargs='?',
                        default='a_iiwa',
                        type=str,
                        choices=None,
                        help='robot name',
                        metavar=None)
    parser.add_argument('-c',
                        '--camera_name',
                        action='store',
                        nargs='?',
                        default='a_phoxi_m_camera',
                        type=str,
                        choices=None,
                        help='camera name',
                        metavar=None)
    parser.add_argument('-n',
                        '--ns',
                        action='store',
                        nargs='?',
                        default='',
                        type=str,
                        choices=None,
                        help='namespace',
                        metavar=None)
    args = parser.parse_args()

    speed = 0.1
    with InteractiveRoutines(args.robot_name,
                             args.camera_name, speed, args.ns) as routines:
        routines.run()
