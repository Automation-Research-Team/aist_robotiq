#!/usr/bin/env python

import sys
import os
import copy
import rospy
import argparse
import moveit_msgs.msg
import geometry_msgs.msg

from math import radians, degrees
from tf import transformations as tfs

from std_msgs.msg import String
from std_srvs.srv import Empty
from std_srvs.srv import Trigger
from o2as_phoxi_camera.srv import GetFrame
from easy_handeye.srv import TakeSample, RemoveSample, ComputeCalibration

from o2as_routines.base import O2ASBaseRoutines
from aist_routines.base import AISTBaseRoutines

# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError

initposes = {
    'o2as': {
        'a_phoxi_m_camera': {
            'a_bot': [
                0.10, -0.10, 0.35, radians(  0), radians( 90), radians(0)],
            'b_bot': [
                0.10,  0.10, 0.35, radians(  0), radians( 90), radians(0)],
            'c_bot': [
                -0.30, 0.00, 0.35, radians(  0), radians( 90), radians(0)],
        },
        'a_bot_camera': {
            'a_bot': [
                0.00, -0.20, 0.35, radians(-90), radians( 90), radians(0)],
        },
    },

    'aist': {
        'a_phoxi_m_camera': {
            'a_bot': [
                0.10,  0.00, 0.35, radians(  0), radians( 90), radians(0)],
            'b_bot': [
                0.10,  0.10, 0.35, radians(  0), radians( 90), radians(0)],
        },
    },

    'ur5e': {
        'a_phoxi_m_camera': {
            'c_bot': [
                0.10, -0.20, 0.35, radians(  0), radians( 90), radians(0)],
            'd_bot': [
                0.10,  0.10, 0.35, radians(  0), radians( 90), radians(0)],
        },
    },
}

# Poses taken during handeye calibration
keyposes = {
    'o2as': {
        'a_phoxi_m_camera': {
            'a_bot': [
                [0.15, -0.20, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15, -0.10, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15,  0.00, 0.25, radians( 30), radians( 25), radians(0)],

                [0.15,  0.00, 0.35, radians( 30), radians( 25), radians(0)],
                [0.15, -0.10, 0.35, radians( 30), radians( 25), radians(0)],
                [0.15, -0.20, 0.35, radians( 30), radians( 25), radians(0)],
            ],

            'b_bot': [
                # configulation for real
                [0.30,  0.15, 0.18, radians( 30), radians( 25), radians(0)],
                [0.30,  0.00, 0.18, radians( 30), radians( 25), radians(0)],
                [0.30, -0.15, 0.18, radians( 30), radians( 25), radians(0)],

                [0.30, -0.10, 0.28, radians( 30), radians( 25), radians(0)],
                [0.30,  0.00, 0.28, radians( 30), radians( 25), radians(0)],
                [0.30,  0.10, 0.28, radians( 30), radians( 25), radians(0)],

                # [0.20,  0.15, 0.20, radians( 30), radians( 25), radians(0)],
                # [0.20,  0.00, 0.20, radians( 30), radians( 25), radians(0)],
                # [0.20, -0.15, 0.20, radians(  0), radians( 25), radians(0)],

                # [0.15, -0.10, 0.10, radians(  0), radians( 25), radians(0)],
                # [0.15,  0.05, 0.10, radians( 30), radians( 25), radians(0)],
                # [0.15,  0.20, 0.10, radians( 30), radians( 25), radians(0)],
            ],

            'c_bot': [
                [0.03, -0.10, 0.19, radians( 30), radians( 25), radians(0)],
                [0.03,  0.00, 0.19, radians( 30), radians( 25), radians(0)],
                [0.03,  0.10, 0.19, radians( 30), radians( 25), radians(0)],
            ]
        },

        'a_bot_camera': {
            'a_bot': [
                [-0.05, -0.20, 0.20, radians(  0), radians( 60), radians(  0)],
                [-0.05, -0.17, 0.20, radians(  0), radians( 75), radians(  0)],
                [-0.05, -0.15, 0.20, radians(  0), radians( 90), radians(  0)],
                [-0.05, -0.10, 0.20, radians(  0), radians(100), radians(  0)],
                [-0.05, -0.05, 0.20, radians(  0), radians(110), radians(  0)],
                [-0.05, -0.05, 0.20, radians(-90), radians( 70), radians(-90)],
                [-0.05, -0.12, 0.20, radians(-90), radians( 85), radians(-90)],
                [-0.05, -0.14, 0.20, radians(-90), radians(105), radians(-90)],
                [-0.05, -0.16, 0.20, radians(-90), radians(120), radians(-90)],
            ]
        },
    },

    'aist': {
        'a_phoxi_m_camera': {
            'a_bot': [
                [0.15,  0.00, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.20, 0.16, radians( 30), radians( 25), radians(0)],

                [0.15,  0.20, 0.30, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.30, radians( 30), radians( 25), radians(0)],
                [0.15,  0.00, 0.30, radians( 30), radians( 25), radians(0)],
            ],

            'b_bot': [
                [0.15,  0.20, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.00, 0.16, radians(  0), radians( 25), radians(0)],

                [0.15,  0.00, 0.25, radians(  0), radians( 25), radians(0)],
                [0.15,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15,  0.20, 0.25, radians( 30), radians( 25), radians(0)],

                # [0.25,  0.20, 0.25, radians( 30), radians( 25), radians(0)],
                # [0.25,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
                # [0.25,  0.00, 0.25, radians(  0), radians( 25), radians(0)],

                # [0.25,  0.00, 0.16, radians(  0), radians( 25), radians(0)],
                # [0.25,  0.10, 0.16, radians( 30), radians( 25), radians(0)],
                # [0.25,  0.20, 0.16, radians( 30), radians( 25), radians(0)],
            ],
        },
    },

    'ur5e': {
        'a_phoxi_m_camera': {
            'c_bot': [
                [0.15,  0.00, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15, -0.05, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.05, 0.16, radians( 30), radians( 25), radians(0)],

                [0.15,  0.05, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15, -0.00, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15, -0.15, 0.25, radians( 30), radians( 25), radians(0)],
            ],

            'd_bot': [
                [0.15,  0.20, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.16, radians( 30), radians( 25), radians(0)],
                [0.15,  0.00, 0.16, radians( 30), radians( 25), radians(0)],

                [0.15,  0.00, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
                [0.15,  0.20, 0.25, radians( 30), radians( 25), radians(0)],
            ],
        },

        'a_bot_camera': {
            'a_bot': [
                [-0.05, -0.20, 0.20, radians(  0), radians( 60), radians(  0)],
                [-0.05, -0.17, 0.20, radians(  0), radians( 75), radians(  0)],
                [-0.05, -0.15, 0.20, radians(  0), radians( 90), radians(  0)],
                [-0.05, -0.10, 0.20, radians(  0), radians(100), radians(  0)],
                [-0.05, -0.05, 0.20, radians(  0), radians(110), radians(  0)],
                [-0.05, -0.05, 0.20, radians(-90), radians( 70), radians(-90)],
                [-0.05, -0.12, 0.20, radians(-90), radians( 85), radians(-90)],
                [-0.05, -0.14, 0.20, radians(-90), radians(105), radians(-90)],
                [-0.05, -0.16, 0.20, radians(-90), radians(120), radians(-90)],
            ]
        },
    },
}


######################################################################
#  global functions                                                  #
######################################################################
def format_pose(pose):
    rpy = map(
        degrees,
        tfs.euler_from_quaternion((
            pose.orientation.x, pose.orientation.y, pose.orientation.z,
            pose.orientation.w
        )))
    return "[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]".format(
        pose.position.x, pose.position.y, pose.position.z, rpy[0], rpy[1],
        rpy[2])


######################################################################
#  class HandEyeCalibrationRoutines                                  #
######################################################################
class HandEyeCalibrationRoutines:
    def __init__(self, routines, camera_name, robot_name, speed, sleep_time,
                 needs_calib):
        self.routines = routines
        self.camera_name = camera_name
        self.speed = speed
        self.sleep_time = sleep_time

        if self.routines.use_real_robot and needs_calib:
            cs = "/{}/".format(camera_name)
            self.start_acquisition = rospy.ServiceProxy(
                cs + "start_acquisition", Trigger)
            self.stop_acquisition = rospy.ServiceProxy(cs + "stop_acquisition",
                                                       Trigger)
        else:
            self.start_acquisition = False
            self.stop_acquisition = False

        if needs_calib:
            ns = "/{}_from_{}/".format(camera_name, robot_name)
            self.take_sample = rospy.ServiceProxy(ns + "take_sample",
                                                  TakeSample)
            self.get_sample_list = rospy.ServiceProxy(ns + "get_sample_list",
                                                      TakeSample)
            self.remove_sample = rospy.ServiceProxy(ns + "remove_sample",
                                                    RemoveSample)
            self.compute_calibration = rospy.ServiceProxy(
                ns + "compute_calibration", ComputeCalibration)
            self.save_calibration = rospy.ServiceProxy(ns + "save_calibration",
                                                       Empty)
        else:
            self.take_sample = False
            self.get_sample_list = False
            self.remove_sample = False
            self.compute_calibration = False
            self.save_calibration = False

        ## Initialize `moveit_commander`
        self.group_name = robot_name
        group = self.routines.groups[self.group_name]

        # Set `_ee_link` as end effector wrt `_base_link` of the robot
        #group.set_planning_frame("workspace_center")
        group.set_pose_reference_frame("workspace_center")
        group.set_end_effector_link(robot_name + "_ee_link")

        # Logging
        print("==== Planning frame:       %s" % group.get_planning_frame())
        print("==== Pose reference frame: %s" %
              group.get_pose_reference_frame())
        print("==== End effector link:    %s" % group.get_end_effector_link())

    def go_home(self):
        self.routines.go_to_named_pose("home", self.group_name)

    def save_image(self, file_name):
        img_msg = rospy.wait_for_message("/aruco_tracker/result",
                                         sensor_msgs.msg.Image,
                                         timeout=10.0)
        bridge = CvBridge()
        cv2.imwrite(file_name, bridge.imgmsg_to_cv2(img_msg, "bgr8"))

    def move(self, pose):
        group = self.routines.groups[self.group_name]
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.header.frame_id = group.get_pose_reference_frame()
        poseStamped.pose.position.x = pose[0]
        poseStamped.pose.position.y = pose[1]
        poseStamped.pose.position.z = pose[2]
        poseStamped.pose.orientation = geometry_msgs.msg.Quaternion(
            *tfs.quaternion_from_euler(pose[3], pose[4], pose[5]))
        print("     move to " + format_pose(poseStamped.pose))
        [all_close, move_success] \
          = self.routines.go_to_pose_goal(
                                self.group_name, poseStamped, self.speed,
                                end_effector_link=group.get_end_effector_link(),
                                move_lin=False)
        poseReached = self.routines.listener.transformPose(
            group.get_pose_reference_frame(), group.get_current_pose())
        print("  reached to " + format_pose(poseReached.pose))
        return move_success

    def move_to_subpose(self, subpose, keypose_num, subpose_num):
        if not self.move(subpose):
            return False

        if self.start_acquisition:
            self.start_acquisition()

            try:
                self.save_image("aruco_result-{:0=2}-{:0=2}.jpeg".format(
                    keypose_num, subpose_num))
            except CvBridgeError, e:
                print(e)
            except rospy.ROSException, e:
                print(e)

        rospy.sleep(self.sleep_time)

        success = True

        if self.take_sample:
            try:
                self.take_sample()
                sample_list = self.get_sample_list()
                n = len(sample_list.samples.hand_world_samples.transforms)
                print(
                    "  took {} (hand-world, camera-marker) samples").format(n)
            except rospy.ServiceException as e:
                print "Service call failed: %s" % e
                success = False

        if self.stop_acquisition:
            self.stop_acquisition()

        return success

    def move_to_subposes(self, pose, keypose_num):
        subpose = copy.deepcopy(pose)
        roll = subpose[3]
        for i in range(3):
            print("\n--- Subpose [{}/5]: Try! ---".format(i + 1))
            if self.move_to_subpose(subpose, keypose_num, i + 1):
                print("--- Subpose [{}/5]: Succeeded. ---".format(i + 1))
            else:
                print("--- Subpose [{}/5]: Failed. ---".format(i + 1))
            subpose[3] -= radians(30)

        subpose[3] = roll - radians(30)
        subpose[4] += radians(15)

        for i in range(2):
            print("\n--- Subpose [{}/5]: Try! ---".format(i + 4))
            if self.move_to_subpose(subpose, keypose_num, i + 4):
                print("--- Subpose [{}/5]: Succeeded. ---".format(i + 4))
            else:
                print("--- Subpose [{}/5]: Failed. ---".format(i + 4))
            subpose[4] -= radians(30)

        # ### How to define poses/positions for calibration
        # # 1. From rostopic echo /joint_states (careful with the order of the joints)
        # joint_pose = [-0.127, 0.556, 0.432, -1.591,  0.147, -0.285]

        # # 2. From Rviz, ee frame position after planning (Open TF Frames, unfold the frame a_bot_ee_link)
        # poseStamped.pose.position.x = -0.16815
        # poseStamped.pose.position.y = -0.10744
        # poseStamped.pose.position.z = 1.1898
        # poseStamped.pose.orientation.x = -0.531
        # poseStamped.pose.orientation.y = 0.5318
        # poseStamped.pose.orientation.z = 0.46652
        # poseStamped.pose.orientation.w = 0.46647

        # # 3. Rotate an orientation using TF quaternions
        # quaternion_0 = tf_conversions.transformations.quaternion_from_euler(
        #       pose[3], pose[4], pose[5])
        # q_rotate_30_in_y = tf_conversions.transformations.quaternion_from_euler(0, pi/6, 0)
        # q_rotated = tf_conversions.transformations.quaternion_multiply(quaternion_0, q_rotate_30_in_y)
        # poseStamped.pose.orientation = geometry_msgs.msg.Quaternion(*q_rotated)

    def run(self, initpose, keyposes):
        if self.stop_acquisition:
            self.stop_acquisition()

        if self.get_sample_list:
            n_samples = len(
                self.get_sample_list().samples.hand_world_samples.transforms)
            if 0 < n_samples:
                for _ in range(n_samples):
                    self.remove_sample(0)

        # Reset pose
        self.go_home()
        self.move(initpose)

        # Collect samples over pre-defined poses
        for i, keypose in enumerate(keyposes):
            print("\n*** Keypose [{}/{}]: Try! ***".format(
                i + 1, len(keyposes)))
            if self.camera_name == "a_bot_camera":
                self.move_to(keypose, i + 1, 1)
            else:
                self.move_to_subposes(keypose, i + 1)
            print("*** Keypose [{}/{}]: Completed. ***".format(
                i + 1, len(keyposes)))

        if self.compute_calibration:
            self.compute_calibration()
            self.save_calibration()

        # Reset pose
        #self.move(initpose)
        self.go_home()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Do hand-eye calibration')
    parser.add_argument('-C',
                        '--config',
                        action='store',
                        nargs='?',
                        default='aist',
                        type=str,
                        choices=None,
                        help='configuration name',
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
    parser.add_argument('-r',
                        '--robot_name',
                        action='store',
                        nargs='?',
                        default='b_bot',
                        type=str,
                        choices=None,
                        help='robot name',
                        metavar=None)
    parser.add_argument('-v',
                        '--visit',
                        action='store_true',
                        help='only visit calibration points')

    args = parser.parse_args()

    try:
        if args.config == 'o2as':
            base_routines = O2ASBaseRoutines()
        else:
            base_routines = AISTBaseRoutines({args.robot_name})

        assert (args.camera_name in {"a_phoxi_m_camera", "a_bot_camera"})
        assert (args.robot_name  in {"a_bot", "b_bot", "c_bot", "d_bot"})

        speed = 1
        sleep_time = 1
        routines = HandEyeCalibrationRoutines(base_routines,
                                              args.camera_name,
                                              args.robot_name,
                                              speed, sleep_time,
                                              not args.visit)

        print("=== Calibration started for {} + {} ===".format(
            args.camera_name, args.robot_name))
        routines.run(initposes[args.config][args.camera_name][args.robot_name],
                     keyposes[ args.config][args.camera_name][args.robot_name])
        print("=== Calibration completed for {} + {} ===".format(
            args.camera_name, args.robot_name))

    except Exception as ex:
        print(ex.message)
