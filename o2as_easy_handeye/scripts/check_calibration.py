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
class VisitRoutines:
    """Wrapper of MoveGroupCommander specific for this script"""

    def __init__(self, routines, camera_name, robot_name, orientations):
        self.routines = routines
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

    def move(self, speed):
        self.routines.start_acquisition(self.camera_name)
        position = rospy.wait_for_message("/aruco_tracker/position",
                                          gmsg.PointStamped, 10)
        self.routines.stop_acquisition(self.camera_name)

        poseStamped = gmsg.PoseStamped()
        poseStamped.header.frame_id = self.group.get_pose_reference_frame()
        poseStamped.pose = gmsg.Pose(
            position.point,
            gmsg.Quaternion(*tfs.quaternion_from_euler(*self.orientations)))
        poseStamped.pose.position.z += 0.05
        self.routines.go_to_pose_goal(self.group.get_name(), poseStamped,
                                      speed, move_lin=False)
        rospy.sleep(1)

        poseStamped.pose.position.z -= 0.05
        print("  move to " + self.format_pose(poseStamped))
        res = self.routines.go_to_pose_goal(self.group.get_name(), poseStamped,
                                            speed, move_lin=True)
        print("  reached " + self.format_pose(res.current_pose))

    def go_home(self):
        self.routines.go_to_named_pose("home", self.group.get_name())

    def run(self, speed):
        self.routines.stop_acquisition(self.camera_name)
        self.go_home()

        while True:
            try:
                key = raw_input(">> ")
                if key == 'q':
                    break
                self.move(speed)
            except Exception as ex:
                self.routines.stop_acquisition(self.camera_name)
                print ex.message
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return

        self.go_home()

    def format_pose(self, poseStamped):
        listener = TransformListener()  # Needs Listener but TransformerROS.
        pose = listener.transformPose(self.group.get_pose_reference_frame(),
                                      poseStamped).pose
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
    parser = argparse.ArgumentParser(description='Check hand-eye calibration')
    parser.add_argument('-C',
                        '--config',
                        action='store',
                        nargs='?',
                        default='pgrp',
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
                        default='a_bot',
                        type=str,
                        choices=None,
                        help='robot name',
                        metavar=None)

    args = parser.parse_args()
    print(args)

    if args.config == 'o2as':
        base_routines = O2ASBaseRoutines()
    else:
        base_routines = AISTBaseRoutines()
    camera_name = args.camera_name
    robot_name = args.robot_name

    assert (camera_name in {"a_phoxi_m_camera", "a_bot_camera"})
    assert (robot_name in {"a_bot", "b_bot", "c_bot", "d_bot"})

    routines = VisitRoutines(base_routines, camera_name, robot_name,
                             orientations[args.config][args.robot_name])
    speed = 0.05
    routines.run(speed)
