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
    'a_bot': [-0.10, 0.00, 0.020, radians(  0), radians( 90), radians( 90)],
    'b_bot': [ 0.10, 0.00, 0.015, radians(  0), radians( 90), radians(-90)],
    'c_bot': [-0.10, 0.00, 0.015, radians(  0), radians( 90), radians( 90)],
    'd_bot': [-0.30, 0.00, 0.015, radians(  0), radians( 90), radians(  0)],
}


######################################################################
#  class ToolCalibrationRoutines                                     #
######################################################################
class ToolCalibrationRoutines:
    def __init__(self, routines, robot_name, camera_name, speed):
        self.routines = routines
        self.camera_name = camera_name
        self.speed = speed
        self.refpose = refposes[robot_name]

        ## Initialize `moveit_commander`
        self.group = moveit_commander.MoveGroupCommander(robot_name)

        # Set `_ee_link` as end effector wrt `_base_link` of the robot
        self.group.set_pose_reference_frame('workspace_center')
        (_, gripper_base_link, gripper_tip_link, _) = \
            self.routines.get_gripper_info(robot_name)
        # if robot_name == 'b_bot':
        #     gripper_base_link = robot_name + '_single_suction_gripper_base_link'
        #     gripper_tip_link  = robot_name + '_single_suction_gripper_pad_link'
        # elif robot_name == 'd_bot':
        #     gripper_base_link = robot_name + '_dual_suction_gripper_base_link'
        #     gripper_tip_link  = robot_name + '_dual_suction_gripper_pad_link'
        # else:
        #     gripper_base_link = robot_name + '_robotiq_85_base_link'
        #     gripper_tip_link  = robot_name + '_robotiq_85_tip_link'
        print("gripper_base_link = " + gripper_base_link)
        print("gripper_tip_link  = " + gripper_tip_link)
        self.gripper_base_link = gripper_base_link
        self.group.set_end_effector_link(gripper_tip_link)

        # Logging
        print("==== Planning frame:       %s" %
              self.group.get_planning_frame())
        print("==== Pose reference frame: %s" %
              self.group.get_pose_reference_frame())
        print("==== End effector link:    %s" %
              self.group.get_end_effector_link())

        self.listener = TransformListener()
        now = rospy.Time.now()
        self.listener.waitForTransform(gripper_base_link, gripper_tip_link,
                                       now, rospy.Duration(10))
        self.D0 = self.listener.fromTranslationRotation(
            *self.listener.lookupTransform(gripper_base_link,
                                           gripper_tip_link, now))
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

    def go_home(self):
        self.routines.go_to_named_pose('home', self.group.get_name())

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
        print("   move to " + self.format_pose(poseStamped))
        res = self.routines.go_to_pose_goal(
                  self.group.get_name(), poseStamped, self.speed,
                  end_effector_link=self.group.get_end_effector_link(),
                  move_lin=False)
        rospy.sleep(1)
        print("   reached " + self.format_pose(res.current_pose))
        return res.success

    def rolling_motion(self):
        pose = copy.deepcopy(self.refpose)
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
        pose = copy.deepcopy(self.refpose)
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
        pose = copy.deepcopy(self.refpose)
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

    def pregrasp(self):
        self.routines.pregrasp(self.group.get_name())

    def grasp(self):
        self.routines.grasp(self.group.get_name())

    def release(self):
        self.routines.release(self.group.get_name())

    def pick(self):
        print("   pick at " + self.format_pose(self.pick_pose))
        self.routines.pick(self.group.get_name(), self.pick_pose)

    def place(self):
        print("  place at " + self.format_pose(self.pick_pose))
        self.routines.place(self.group.get_name(), self.place_pose)

    def search_graspability(self):
        (poses, rotipz, gscore, success) = \
            self.routines.search_graspability(self.group.get_name(),
                                              self.camera_name, 4, 0)
        for gs in gscore:
            print(str(gs))
        print(str(poses))

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
            elif key == 'r':
                self.move(self.refpose)
                self.rolling_motion()
            elif key == 'p':
                self.move(self.refpose)
                self.pitching_motion()
            elif key == 'y':
                self.move(self.refpose)
                self.yawing_motion()
            elif key == 't':
                self.move(self.refpose)
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
                    self.refpose[0] += 0.01
                elif axis == 'Y    ':
                    self.refpose[1] += 0.01
                elif axis == 'Z    ':
                    self.refpose[2] += 0.01
                elif axis == 'Pitch':
                    self.pitch += radians(0.5)
                else:
                    self.yaw += radians(0.5)
                self.move(self.refpose)
            elif key == '-':
                if axis == 'X    ':
                    self.refpose[0] -= 0.01
                elif axis == 'Y    ':
                    self.refpose[1] -= 0.01
                elif axis == 'Z    ':
                    self.refpose[2] -= 0.01
                elif axis == 'Pitch':
                    self.pitch -= radians(0.5)
                else:
                    self.yaw -= radians(0.5)
                self.move(self.refpose)
            elif key == 'd':
                self.print_tip_link()
            elif key == 'pregrasp':
                self.pregrasp()
            elif key == 'grasp':
                self.grasp()
            elif key == 'release':
                self.release()
            elif key == 'pick':
                self.pick()
            elif key == 'place':
                self.place()
            elif key == 's':
                self.search_graspability()
            elif key == 'h':
                self.go_home()
            else:
                if axis == 'X    ':
                    self.refpose[0] = float(key)
                elif axis == 'Y    ':
                    self.refpose[1] = float(key)
                elif axis == 'Z    ':
                    self.refpose[2] = float(key)
                elif axis == 'Pitch':
                    self.pitch = radians(float(key))
                else:
                    self.yaw = radians(float(key))
                self.move(self.refpose)

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
    parser.add_argument('-C',
                        '--config',
                        action='store',
                        nargs='?',
                        default='aist',
                        type=str,
                        choices=None,
                        help='configuration name',
                        metavar=None)
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

        if args.config == 'o2as':
            base_routines = O2ASBaseRoutines()
        else:
            base_routines = AISTBaseRoutines()

        speed = 0.1
        routines = ToolCalibrationRoutines(base_routines, args.robot_name,
                                           args.camera_name, speed)

        routines.run()

    except Exception as ex:
        print(ex.message)
