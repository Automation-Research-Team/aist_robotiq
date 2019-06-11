#!/usr/bin/env python

import sys
import os
import rospy
import argparse
from math import radians, degrees
from std_srvs.srv import Trigger
from geometry_msgs import msg as gmsg
from tf import TransformListener, transformations as tfs

import moveit_commander

from o2as_routines.base import O2ASBaseRoutines
from aist_routines.base import AISTBaseRoutines

orientations = {
    'o2as': {
        'a_bot': [
            radians(-90), radians( 90), radians(0)],
        'b_bot': [
            radians(  0), radians( 90), radians(0)],
        'c_bot': [
            radians(  0), radians( 90), radians(0)],
    },

    'aist': {
        'a_bot': [
            radians(-90), radians( 90), radians(0)],
        'b_bot': [
            radians(  0), radians( 90), radians(0)],
    },

    'pgrp': {
        'a_bot': [
            radians(-90), radians( 90), radians(0)],
        'b_bot': [
            radians(  0), radians( 90), radians(0)],
    },

    'ur5e': {
        'c_bot': [
            radians(-90), radians( 90), radians(0)],
        'd_bot': [
            radians(  0), radians( 90), radians(0)],
    },
}

######################################################################
#  class VisitRoutines                                               #
######################################################################
class VisitRoutines(AISTBaseRoutines):
    """Wrapper of MoveGroupCommander specific for this script"""

    def __init__(self, camera_name, robot_name, orientations):
        super(VisitRoutines, self).__init__()

        self.camera_name = camera_name

        ## Initialize `moveit_commander`
        self.group = moveit_commander.MoveGroupCommander(robot_name)
        self.group.set_pose_reference_frame("workspace_center")

        # Logging
        print("==== Planning frame:       %s" %
              self.group.get_planning_frame())
        print("==== Pose reference frame: %s" %
              self.group.get_pose_reference_frame())
        print("==== End effector link:    %s" %
              self.group.get_end_effector_link())

        self.orientations = orientations

    @property
    def robot_name(self):
        return self.group.get_name()

    def move(self, speed):
        self.start_acquisition(self.camera_name)
        position = rospy.wait_for_message("/aruco_tracker/position",
                                          gmsg.PointStamped, 10)
        self.stop_acquisition(self.camera_name)

        poseStamped = gmsg.PoseStamped()
        poseStamped.header.frame_id = self.group.get_pose_reference_frame()
        poseStamped.pose = gmsg.Pose(
            position.point,
            gmsg.Quaternion(*tfs.quaternion_from_euler(*self.orientations)))
        poseStamped.pose.position.z += 0.05
        self.go_to_pose_goal(self.robot_name, poseStamped, speed, move_lin=True)
        rospy.sleep(1)

        poseStamped.pose.position.z -= 0.05
        self.go_to_pose_goal(self.robot_name, poseStamped, speed, move_lin=True)

    def go_home(self):
        self.go_to_named_pose("home", self.robot_name)

    def run(self, speed):
        self.stop_acquisition(self.camera_name)
        self.go_home()

        while True:
            try:
                key = raw_input(">> ")
                if key == 'q':
                    break
                self.move(speed)
            except Exception as ex:
                self.stop_acquisition(self.camera_name)
                print ex.message
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return

        self.go_home()

######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Check hand-eye calibration')
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

    args = parser.parse_args()
    print(args)

    assert (args.camera_name in {"a_phoxi_m_camera", "a_bot_camera"})
    assert (args.robot_name  in {"a_bot", "b_bot", "c_bot", "d_bot"})

    routines = VisitRoutines(args.camera_name, args.robot_name,
                             orientations[args.config][args.robot_name])
    speed = 0.05
    routines.run(speed)
    rospy.signal_shutdown("Calibration completed.")
