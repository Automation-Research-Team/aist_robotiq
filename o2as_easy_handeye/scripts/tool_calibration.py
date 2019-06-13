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
from aist_routines.base import AISTBaseRoutines

refposes = {
#    'a_bot': [-0.10, 0.00, 0.20,  radians(-90), radians(-90), radians(180)],
    'a_bot': [0.00, 0.00, 0.015, radians(  0), radians( 90), radians( 90)],
    'b_bot': [0.00, 0.00, 0.015, radians(  0), radians( 90), radians(-90)],
    'c_bot': [0.00, 0.00, 0.015, radians(  0), radians( 90), radians( 90)],
    'd_bot': [0.00, 0.00, 0.015, radians(  0), radians( 90), radians(  0)],
}


######################################################################
#  class ToolCalibrationRoutines                                     #
######################################################################
class ToolCalibrationRoutines(AISTBaseRoutines):
    def __init__(self, robot_name, camera_name, speed):
        super(ToolCalibrationRoutines, self).__init__()

        self.camera_name = camera_name
        self.speed       = speed
        self.refpose     = refposes[robot_name]
        self.goalpose    = copy.deepcopy(self.refpose)

        ## Initialize `moveit_commander`
        self.group = moveit_commander.MoveGroupCommander(robot_name)

        self.gripper_base_link = self.gripper(robot_name).base_link
        self.gripper_tip_link  = self.gripper(robot_name).tip_link
        self.group.set_pose_reference_frame('workspace_center')
        self.group.set_end_effector_link(self.gripper_tip_link)

        # Logging
        print("gripper_base_link = " + self.gripper_base_link)
        print("gripper_tip_link  = " + self.gripper_tip_link)
        print("==== Planning frame:       %s" %
              self.group.get_planning_frame())
        print("==== Pose reference frame: %s" %
              self.group.get_pose_reference_frame())
        print("==== End effector link:    %s" %
              self.group.get_end_effector_link())

        self.listener = TransformListener()
        now = rospy.Time.now()
        self.listener.waitForTransform(self.gripper_base_link,
                                       self.gripper_tip_link,
                                       now, rospy.Duration(10))
        self.D0 = self.listener.fromTranslationRotation(
            *self.listener.lookupTransform(self.gripper_base_link,
                                           self.gripper_tip_link, now))
        self.pitch = 0.0
        self.yaw   = 0.0

        self.pick_pose = gmsg.PoseStamped()
        self.pick_pose.header.frame_id = self.group.get_pose_reference_frame()
        self.pick_pose.pose = gmsg.Pose(gmsg.Point(-0.1, 0.1, 0.01),
                                        gmsg.Quaternion(
                                            *tfs.quaternion_from_euler(
                                                radians(15), 0, 0)))

        self.place_pose = gmsg.PoseStamped()
        self.place_pose.header.frame_id  = self.group.get_pose_reference_frame()
        self.place_pose.pose = gmsg.Pose(gmsg.Point(0.1, 0, 0.01),
                                         gmsg.Quaternion(
                                             *tfs.quaternion_from_euler(
                                                 0, 0, 0)))

    @property
    def robot_name(self):
        return self.group.get_name()

    def go_home(self):
        self.go_to_named_pose('home', self.robot_name)

    def correct_end_effector_link(self):
        D = tfs.concatenate_matrices(
            self.listener.fromTranslationRotation(
                (0, 0, 0), tfs.quaternion_from_euler(0, self.pitch, self.yaw)),
            self.D0)
        print('  trns = {}, rot = {}'.format(tfs.translation_from_matrix(D),
                                             tfs.quaternion_from_matrix(D)))
        rate = rospy.Rate(10.0)
        self.broadcaster.sendTransform(
            tfs.translation_from_matrix(D), tfs.quaternion_from_matrix(D),
            rospy.Time.now(),
            self.group.get_end_effector_link() + '_corrected',
            self.gripper_base_link)
        rate.sleep()

    def move(self, pose):
        R = self.listener.fromTranslationRotation(
                (0, 0, 0), tfs.quaternion_from_euler(0, self.pitch, self.yaw))
        T = tfs.concatenate_matrices(
            self.listener.fromTranslationRotation(
                (pose[0], pose[1], pose[2]),
                tfs.quaternion_from_euler(pose[3], pose[4], pose[5])),
            tfs.inverse_matrix(self.D0), tfs.inverse_matrix(R), self.D0)
        poseStamped = gmsg.PoseStamped()
        poseStamped.header.frame_id = self.group.get_pose_reference_frame()
        poseStamped.pose = gmsg.Pose(
            gmsg.Point(*tfs.translation_from_matrix(T)),
            gmsg.Quaternion(*tfs.quaternion_from_matrix(T)))
        (success, _, _ ) = self.go_to_pose_goal(self.robot_name, poseStamped,
                                                self.speed, move_lin=True,
                                                high_precision=True)
        return success

    def rolling_motion(self):
        pose = copy.deepcopy(self.goalpose)
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
        pose = copy.deepcopy(self.goalpose)
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
        pose = copy.deepcopy(self.goalpose)
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
                tfs.quaternion_from_euler(0, self.pitch, self.yaw))
        D   = tfs.concatenate_matrices(R, self.D0)
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

        while True:
            prompt = axis + '[p=' + str(degrees(self.pitch)) + \
                            ',y=' + str(degrees(self.yaw)) + '] >> '
            key = raw_input(prompt)
            if key == 'q':
                break
            elif key == 'o':
                self.move(self.refpose)
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
                    self.goalpose[0] += 0.01
                elif axis == 'Y    ':
                    self.goalpose[1] += 0.01
                elif axis == 'Z    ':
                    self.goalpose[2] += 0.01
                elif axis == 'Pitch':
                    self.pitch += radians(0.5)
                else:
                    self.yaw += radians(0.5)
                self.move(self.goalpose)
            elif key == '-':
                if axis == 'X    ':
                    self.goalpose[0] -= 0.01
                elif axis == 'Y    ':
                    self.goalpose[1] -= 0.01
                elif axis == 'Z    ':
                    self.goalpose[2] -= 0.01
                elif axis == 'Pitch':
                    self.pitch -= radians(0.5)
                else:
                    self.yaw -= radians(0.5)
                self.move(self.goalpose)
            elif key == 'd':
                self.print_tip_link()
            elif key == 'pregrasp':
                self.pregrasp(self.robot_name)
            elif key == 'grasp':
                self.grasp(self.robot_name)
            elif key == 'release':
                self.release(self.robot_name)
            elif key == 'pick':
                print("   pick at " + self.format_pose(self.pick_pose))
                self.pick(self.robot_name, self.pick_pose)
            elif key == 'place':
                print("  place at " + self.format_pose(self.pick_pose))
                self.place(self.robot_name, self.place_pose)
            elif key == 'c':
                return self.create_mask_image(self.camera_name,
                                              int(raw_input("  #bins? ")))
            elif key == 's':
                (poses, rotipz, gscore, success) = \
                    self.search_graspability(self.robot_name,
                                             self.camera_name, 4, 0)
                for gs in gscore:
                    print(str(gs))
                print(str(poses))
            elif key == 'h':
                self.go_home()
            else:
                if axis == 'X    ':
                    self.goalpose[0] = float(key)
                elif axis == 'Y    ':
                    self.goalpose[1] = float(key)
                elif axis == 'Z    ':
                    self.goalpose[2] = float(key)
                elif axis == 'Pitch':
                    self.pitch = radians(float(key))
                else:
                    self.yaw = radians(float(key))
                self.move(self.goalpose)

        # Reset pose
        self.go_home()
        self.print_tip_link()

    def format_pose(self, poseStamped):
        pose = self.listener.transformPose(
            self.group.get_pose_reference_frame(), poseStamped).pose
        rpy = map(
            degrees,
            tfs.euler_from_quaternion([pose.orientation.w, pose.orientation.x,
                                       pose.orientation.y, pose.orientation.z]))
        return "[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]".format(
            pose.position.x, pose.position.y, pose.position.z,
            rpy[0], rpy[1], rpy[2])


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

    try:
        assert (args.robot_name in {'a_bot', 'b_bot', 'c_bot', 'd_bot'})

        speed = 0.1
        routines = ToolCalibrationRoutines(args.robot_name,
                                           args.camera_name, speed)
        routines.run()
    except Exception as ex:
        print(ex.message)
    rospy.signal_shutdown("Calibration completed")
