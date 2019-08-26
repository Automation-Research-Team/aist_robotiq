#!/usr/bin/env python

import sys
import os
import copy
import rospy
import argparse
from geometry_msgs import msg as gmsg

import moveit_commander

from tf import transformations as tfs
from math import radians, degrees
from aist_routines.ur import URRoutines

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
class InteractiveRoutines(URRoutines):
    refposes = {
        'a_bot': [0.00, 0.00, 0.15, radians(  0), radians( 80), radians( 90)],
        'b_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians(-90)],
        'c_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians( 90)],
        'd_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians(  0)],
    }

    def __init__(self, robot_name, camera_name, speed):
        super(InteractiveRoutines, self).__init__()

        self._robot_name   = robot_name
        self._camera_name  = camera_name
        self._speed        = speed
        self._ur_movel     = False
        group = moveit_commander.MoveGroupCommander(robot_name)
        print("=== End effector: %s" % group.get_end_effector_link())

    def go_home(self):
        self.go_to_named_pose('home', self._robot_name)

    def go_back(self):
        self.go_to_named_pose('back', self._robot_name)

    def move(self, pose):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = "workspace_center"
        target_pose.pose = gmsg.Pose(
            gmsg.Point(pose[0], pose[1], pose[2]),
            gmsg.Quaternion(
                *tfs.quaternion_from_euler(pose[3], pose[4], pose[5])))
        print("move to " + self.format_pose(target_pose))
        if self._ur_movel:
            (success, _, current_pose) = self.ur_movel(self._robot_name,
                                                       target_pose,
                                                       velocity=self._speed)
        else:
            (success, _, current_pose) = self.go_to_pose_goal(
                                                self._robot_name, target_pose,
                                                self._speed, move_lin=True)
        print("reached " + self.format_pose(current_pose))
        return success

    def xyz_rpy(self, poseStamped):
        pose = self.listener.transformPose("workspace_center",
                                           poseStamped).pose
        rpy  = tfs.euler_from_quaternion([pose.orientation.x,
                                          pose.orientation.y,
                                          pose.orientation.z,
                                          pose.orientation.w])
        return [pose.position.x, pose.position.y, pose.position.z,
                rpy[0], rpy[1], rpy[2]]

    def run(self):
        # Reset pose
        self.go_home()

        axis = 'Y'

        while not rospy.is_shutdown():
            group = moveit_commander.MoveGroupCommander(self._robot_name)
            print("=== End effector: %s" % group.get_end_effector_link())

            current_pose = self.get_current_pose(self._robot_name)
            prompt = "{:>5}{}{:>9}>> " \
                   .format(axis, self.format_pose(current_pose),
                           "urscript" if self._ur_movel else "moveit")

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
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] = float(key)
                elif axis == 'Y':
                    goal_pose[1] = float(key)
                elif axis == 'Z':
                    goal_pose[2] = float(key)
                elif axis == 'Roll':
                    goal_pose[3] = radians(float(key))
                elif axis == 'Pitch':
                    goal_pose[4] = radians(float(key))
                else:
                    goal_pose[5] = radians(float(key))
                self.move(goal_pose)
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
                (poses, rotipz, gscore, success) = \
                    self.search_graspability(self._robot_name,
                                             self._camera_name, 4, 0)
                for gs in gscore:
                    print(str(gs))
                print(str(poses))
            elif key == 'ur':
                self._ur_movel = not self._ur_movel
            elif key == 'push':
                self.ur_linear_push(self._robot_name, wait=False)
            elif key == 'spiral':
                self.ur_spiral_motion(self._robot_name, wait=False)
            elif key == 'insertion':
                self.ur_insertion(self._robot_name, wait=False)
            elif key == 'hinsertion':
                self.ur_horizontal_insertion(self._robot_name, wait=False)
            elif key == 'spiral':
                self.ur_spiral_motion(self._robot_name, wait=False)
            elif key == 'r':
                self.move(InteractiveRoutines.refposes[self._robot_name])
            elif key == 'h':
                self.go_home()
            elif key == 'b':
                self.go_back()

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
                        default='b_bot',
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
    args = parser.parse_args()

    assert (args.robot_name in {'a_bot', 'b_bot', 'c_bot', 'd_bot'})

    speed = 0.1
    with InteractiveRoutines(args.robot_name,
                             args.camera_name, speed) as routines:
        routines.run()
