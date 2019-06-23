#!/usr/bin/env python

import sys
import os
import copy
import rospy
import argparse
import moveit_msgs.msg
import moveit_commander
from geometry_msgs import msg as gmsg

from tf import TransformListener, transformations as tfs
from math import radians, degrees
from o2as_routines.base import O2ASBaseRoutines
from aist_routines.ur   import URRoutines

refposes = {
#    'a_bot': [-0.10, 0.00, 0.20,  radians(-90), radians(-90), radians(180)],
    'a_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians( 90)],
    'b_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians(-90)],
    'c_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians( 90)],
    'd_bot': [0.00, 0.00, 0.15, radians(  0), radians( 90), radians(  0)],
}


######################################################################
#  class ToolCalibrationRoutines                                     #
######################################################################
class ToolCalibrationRoutines(URRoutines):
    def __init__(self, robot_name, camera_name, speed):
        super(ToolCalibrationRoutines, self).__init__()

        self._robot_name  = robot_name
        self._camera_name = camera_name
        self._speed       = speed
        self._refpose     = refposes[robot_name]
        self._goalpose    = copy.deepcopy(self._refpose)
        self._D0           = self.listener.fromTranslationRotation(
            *self.listener.lookupTransform(self.gripper(robot_name).base_link,
                                           self.gripper(robot_name).tip_link,
                                           rospy.Time(0)))
        self._pitch = 0.0
        self._yaw   = 0.0

        self._pick_pose = gmsg.PoseStamped()
        self._pick_pose.header.frame_id = "workspace_center"
        self._pick_pose.pose = gmsg.Pose(gmsg.Point(-0.1, 0.1, 0.01),
                                         gmsg.Quaternion(
                                             *tfs.quaternion_from_euler(
                                                 radians(15), 0, 0)))

        self._place_pose = gmsg.PoseStamped()
        self._place_pose.header.frame_id  = "workspace_center"
        self._place_pose.pose = gmsg.Pose(gmsg.Point(0.1, 0, 0.01),
                                          gmsg.Quaternion(
                                              *tfs.quaternion_from_euler(
                                                  0, 0, 0)))

    def go_home(self):
        self.go_to_named_pose('home', self._robot_name)

    def move(self, pose):
        R = self.listener.fromTranslationRotation(
                (0, 0, 0),
                tfs.quaternion_from_euler(0, self._pitch, self._yaw))
        T = tfs.concatenate_matrices(
                self.listener.fromTranslationRotation(
                    (pose[0], pose[1], pose[2]),
                    tfs.quaternion_from_euler(pose[3], pose[4], pose[5])),
                tfs.inverse_matrix(self._D0),
                tfs.inverse_matrix(R),
                self._D0)
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = "workspace_center"
        target_pose.pose = gmsg.Pose(
            gmsg.Point(*tfs.translation_from_matrix(T)),
            gmsg.Quaternion(*tfs.quaternion_from_matrix(T)))
        print("move to " + self.format_pose(target_pose))
        (success, _, pose) = self.go_to_pose_goal(self._robot_name,
                                                  target_pose,
                                                  self._speed, move_lin=True,
                                                  high_precision=True)
        print("reached " + self.format_pose(pose))
        return success

    def rolling_motion(self):
        pose = copy.deepcopy(self._goalpose)
        for i in range(4):
            pose[3] += radians(30)
            self.move(pose)
        for i in range(8):
            pose[3] -= radians(30)
            self.move(pose)
        for i in range(4):
            pose[3] += radians(30)
            self.move(pose)

    def pitching_motion(self):
        pose = copy.deepcopy(self._goalpose)
        for i in range(5):
            pose[4] += radians(6)
            self.move(pose)
        for i in range(10):
            pose[4] -= radians(6)
            self.move(pose)
        for i in range(5):
            pose[4] += radians(6)
            self.move(pose)

    def yawing_motion(self):
        pose = copy.deepcopy(self._goalpose)
        pose[3] = radians(-90)
        pose[5] = radians(180)
        for i in range(5):
            pose[4] += radians(6)
            self.move(pose)
        for i in range(10):
            pose[4] -= radians(6)
            self.move(pose)
        for i in range(5):
            pose[4] += radians(6)
            self.move(pose)

    def print_tip_link(self):
        R   = self.listener.fromTranslationRotation(
                (0, 0, 0),
                tfs.quaternion_from_euler(0, self._pitch, self._yaw))
        D   = tfs.concatenate_matrices(R, self._D0)
        xyz = tfs.translation_from_matrix(D)
        q   = tfs.quaternion_from_matrix(D)
        rpy = map(degrees, tfs.euler_from_quaternion(q))
        print(
            '<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'
            .format(xyz, rpy))

    def run(self):
        # Reset pose
        self.go_home()

        axis = 'Pitch'

        while not rospy.is_shutdown():
            prompt = axis + '[p=' + str(degrees(self._pitch)) + \
                            ',y=' + str(degrees(self._yaw)) + '] >> '
            key = raw_input(prompt)

            if key == 'q':
                break
            elif key == 'o':
                self.move(self._refpose)
            elif key == 'r':
                self.rolling_motion()
            elif key == 'p':
                self.pitching_motion()
            elif key == 'y':
                self.yawing_motion()
            elif key == 't':
                self.rolling_motion()
                self.pitching_motion()
                self.yawing_motion()
            elif key == 'X':
                axis = 'X    '
            elif key == 'Y':
                axis = 'Y    '
            elif key == 'Z':
                axis = 'Z    '
            elif key == 'P':
                axis = 'Pitch'
            elif key == 'W':
                axis = 'Yaw  '
            elif key == '+':
                if axis == 'X    ':
                    self._goalpose[0] += 0.01
                elif axis == 'Y    ':
                    self._goalpose[1] += 0.01
                elif axis == 'Z    ':
                    self._goalpose[2] += 0.01
                elif axis == 'Pitch':
                    self._pitch += radians(0.5)
                else:
                    self._yaw += radians(0.5)
                self.move(self._goalpose)
            elif key == '-':
                if axis == 'X    ':
                    self._goalpose[0] -= 0.01
                elif axis == 'Y    ':
                    self._goalpose[1] -= 0.01
                elif axis == 'Z    ':
                    self._goalpose[2] -= 0.01
                elif axis == 'Pitch':
                    self._pitch -= radians(0.5)
                else:
                    self._yaw -= radians(0.5)
                self.move(self._goalpose)
            elif key == 'd':
                self.print_tip_link()
            elif key == 'pregrasp':
                self.pregrasp(self._robot_name)
            elif key == 'grasp':
                self.grasp(self._robot_name)
            elif key == 'release':
                self.release(self._robot_name)
            elif key == 'pick':
                print("   pick at " + self.format_pose(self._pick_pose))
                self.pick(self._robot_name, self._pick_pose)
            elif key == 'place':
                print("  place at " + self.format_pose(self._pick_pose))
                self.place(self._robot_name, self._place_pose)
            elif key == 'c':
                self.create_mask_image(self._camera_name,
                                       int(raw_input("  #bins? ")))
            elif key == 's':
                self.delete_all_markers()
                (poses, rotipz, gscore, success) = \
                    self.search_graspability(self._robot_name,
                                             self._camera_name, 4, 0)
                for gs in gscore:
                    print(str(gs))
                print(str(poses))
            elif key == 'push':
                self.do_linear_push(self._robot_name, wait=False)
            elif key == 'spiral':
                self.do_spiral_motion(self._robot_name, wait=False)
            elif key == 'insertion':
                self.do_insertion(self._robot_name, wait=False)
            elif key == 'hinsertion':
                self.do_horizontal_insertion(self._robot_name, wait=False)
            elif key == 'spiral':
                self.do_spiral_motion(self._robot_name, wait=False)
            elif key == 'h':
                self.go_home()
            else:
                if axis == 'X    ':
                    self._goalpose[0] = float(key)
                elif axis == 'Y    ':
                    self._goalpose[1] = float(key)
                elif axis == 'Z    ':
                    self._goalpose[2] = float(key)
                elif axis == 'Pitch':
                    self._pitch = radians(float(key))
                else:
                    self._yaw = radians(float(key))
                self.move(self._goalpose)

        # Reset pose
        self.go_home()
        self.print_tip_link()

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
    with ToolCalibrationRoutines(args.robot_name,
                                 args.camera_name, speed) as routines:
        routines.run()
