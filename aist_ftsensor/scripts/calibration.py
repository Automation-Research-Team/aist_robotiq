#!/usr/bin/env python

import sys
import os
import rospy
import argparse
from math import radians, degrees
from std_srvs.srv  import Empty, Trigger
from geometry_msgs import msg as gmsg
from tf import TransformListener, transformations as tfs

from aist_routines.base import AISTBaseRoutines

import moveit_commander
import sensor_msgs.msg

"""
initposes = {
    'pgrp' : {
        'a_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians( 90)],
        'b_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians(-90)],
        'c_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians( 90)],
        'd_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians(  0)],
    },
}
"""

# Poses taken during handeye calibration
keyposes = {
    'pgrp' : {
        'a_bot': [
            [0.00, 0.00, 0.3, radians(  0), radians( 90), radians( 90)],
            [0.00, 0.00, 0.3, radians(  0), radians( 45), radians( 90)],
            [0.00, 0.00, 0.3, radians(  0), radians(  0), radians( 90)],
            [0.00, 0.00, 0.3, radians( 45), radians(  0), radians( 90)],
            [0.00, 0.00, 0.3, radians( 90), radians(  0), radians( 90)],
            [0.00, 0.00, 0.3, radians( 90), radians( 45), radians( 90)],
            [0.00, 0.00, 0.3, radians( 90), radians( 90), radians( 90)],
            [0.00, 0.00, 0.3, radians( 90), radians(135), radians( 90)],
            [0.00, 0.00, 0.3, radians( 45), radians(135), radians( 90)],
            [0.00, 0.00, 0.3, radians(  0), radians(135), radians( 90)],
        ],
        'b_bot': [
            [0.00, 0.00, 0.3, radians(  0), radians( 90), radians(-90)],
            [0.00, 0.00, 0.3, radians(  0), radians( 45), radians(-90)],
            [0.00, 0.00, 0.3, radians(  0), radians(  0), radians(-90)],
            [0.00, 0.00, 0.3, radians( 45), radians(  0), radians(-90)],
            [0.00, 0.00, 0.3, radians( 90), radians(  0), radians(-90)],
            [0.00, 0.00, 0.3, radians( 90), radians( 45), radians(-90)],
            [0.00, 0.00, 0.3, radians( 90), radians( 90), radians(-90)],
            [0.00, 0.00, 0.3, radians( 90), radians(135), radians(-90)],
            [0.00, 0.00, 0.3, radians( 45), radians(135), radians(-90)],
            [0.00, 0.00, 0.3, radians(  0), radians(135), radians(-90)],
        ],
        'c_bot': [
            [0.00, 0.00, 0.3, radians(  0), radians( 90), radians( 90)],
        ],
        'd_bot': [
            [0.00, 0.00, 0.3, radians(  0), radians( 90), radians(  0)],
        ],
    },
}


######################################################################
#  class FTCalibrationRoutines                                  #
######################################################################
class FTCalibrationRoutines(AISTBaseRoutines):
    def __init__(self, robot_name, speed, sleep_time, needs_calib):
        super(FTCalibrationRoutines, self).__init__()

        self._robot_name  = robot_name
        self._speed       = speed
        self._sleep_time  = sleep_time

        if needs_calib:
            ns = "/{}_ftsensor/".format(robot_name)
            self.take_sample = rospy.ServiceProxy(ns + "take_sample", Trigger)
            self.compute_calibration = rospy.ServiceProxy(
                ns + "compute_calibration", Trigger)
            self.save_calibration = rospy.ServiceProxy(ns + "save_calibration",
                                                       Trigger)
        else:
            self.take_sample         = None
            self.compute_calibration = None
            self.save_calibration    = None

    def go_home(self):
        self.go_to_named_pose(self._robot_name, "home")

    def move(self, pose):
        poseStamped = gmsg.PoseStamped()
        poseStamped.header.frame_id = "workspace_center"
        poseStamped.pose = gmsg.Pose(
            gmsg.Point(pose[0], pose[1], pose[2]),
            gmsg.Quaternion(
                *tfs.quaternion_from_euler(pose[3], pose[4], pose[5])))
        print("  move to " + self.format_pose(poseStamped))
        (success, _, current_pose) \
            = self.go_to_pose_goal(
                self._robot_name, poseStamped, self._speed,
                end_effector_link=self._robot_name + "_ee_link",
                move_lin=True)
        print("  reached " + self.format_pose(current_pose))
        return success

    def move_to(self, pose, keypose_num, subpose_num):
        success = self.move(pose)
        if not success:
            return False

        if self.take_sample:
            try:
                rospy.sleep(1)  # Wait for the robot settling.
                rospy.sleep(self._sleep_time)
                res = self.take_sample()
                print("  samples taken: {}").format(res.message)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                success = False

        return success

    """
    def run(self, initpose, keyposes):
    """
    def run(self, keyposes):
        # Reset pose
        self.go_home()
        """
        self.move(initpose)
        """

        # Collect samples over pre-defined poses
        for i, keypose in enumerate(keyposes, 1):
            print("\n*** Keypose [{}/{}]: Try! ***".format(i, len(keyposes)))
            self.move_to(keypose, i, 1)
            print("*** Keypose [{}/{}]: Completed. ***".format(i,
                                                               len(keyposes)))

        if self.compute_calibration:
            res = self.compute_calibration()
            print("  compute calibration: {}").format(res.message)
            res = self.save_calibration()
            print("  save calibration: {}").format(res.message)

        self.go_home()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Do force sensor calibration')
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
    parser.add_argument('-v',
                        '--visit',
                        action='store_true',
                        help='only visit calibration points')

    args = parser.parse_args()

    """
    assert (args.config     in initposes)
    assert (args.robot_name in initposes[args.config])
    """
    assert (args.config     in keyposes)
    assert (args.robot_name in keyposes[args.config])

    speed = 1
    sleep_time = 2
    with FTCalibrationRoutines(args.robot_name,
                                              speed, sleep_time,
                                              not args.visit) as routines:

        print("=== Calibration started for {} ===".format(args.robot_name))
        """
        routines.run(initposes[args.config][args.robot_name],
                     keyposes[ args.config][args.robot_name])
        """
        routines.run(keyposes[ args.config][args.robot_name])
        print("=== Calibration completed for {} ===".format(
                     args.robot_name))
