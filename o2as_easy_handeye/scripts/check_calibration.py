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
        'a_bot': [radians(-90), radians( 90), radians(0)],
        'b_bot': [radians(  0), radians( 90), radians(0)],
        'c_bot': [radians(  0), radians( 90), radians(0)],
    },

    'aist': {
        'a_bot': [radians(-90), radians( 90), radians(0)],
        'b_bot': [radians(  0), radians( 90), radians(0)],
    },

    'pgrp': {
        'a_bot': [radians(-90), radians( 90), radians(0)],
        'b_bot': [radians(  0), radians( 90), radians(0)],
    },

    'ur5e': {
        'c_bot': [radians(-90), radians( 90), radians(0)],
        'd_bot': [radians(  0), radians( 90), radians(0)],
    },
}

######################################################################
#  class VisitRoutines                                               #
######################################################################
class VisitRoutines(AISTBaseRoutines):
    """Wrapper of MoveGroupCommander specific for this script"""

    def __init__(self, robot_name, camera_name, orientations):
        super(VisitRoutines, self).__init__()

        self._robot_name   = robot_name
        self._camera_name  = camera_name
        self._orientations = orientations

        # Logging
        group = moveit_commander.MoveGroupCommander(robot_name)
        print("==== Planning frame:       %s" % group.get_planning_frame())
        print("==== Pose reference frame: %s" %
              group.get_pose_reference_frame())
        print("==== End effector link:    %s" % group.get_end_effector_link())


    def move(self, speed):
        self.trigger_frame(self._camera_name)
        pose = rospy.wait_for_message(self._camera_name + "/aruco_detector/pose",
                                      gmsg.PoseStamped, 10)
        approach_pose = self.effector_target_pose(pose, (0, 0, 0.05))
        print("  move to " + self.format_pose(approach_pose))
        (success, _, current_pose) \
            = self.go_to_pose_goal(self._robot_name, approach_pose,
                                   speed, move_lin=True)
        print("  reached " + self.format_pose(current_pose))
        rospy.sleep(1)
        print("  move to " + self.format_pose(pose))
        (success, _, current_pose) \
            = self.go_to_pose_goal(self._robot_name,
                                   self.effector_target_pose(pose, (0, 0, 0)),
                                   speed, move_lin=True)
        print("  reached " + self.format_pose(current_pose))

    def run(self, speed):
        self.continuous_shot(self._camera_name, False)
        self.go_to_named_pose("home", self._robot_name)

        while True:
            try:
                key = raw_input(">> ")
                if key == 'q':
                    break
                elif key == 'h':
                    self.go_to_named_pose("home", self._robot_name)
                else:
                    self.move(speed)
            except rospy.ROSException as ex:
                print ex.message
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return

        self.go_to_named_pose("home", self._robot_name)

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

    with VisitRoutines(args.robot_name, args.camera_name,
                       orientations[args.config][args.robot_name]) as routines:

        speed = 0.05
        routines.run(speed)
