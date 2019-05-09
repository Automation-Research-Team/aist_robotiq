#!/usr/bin/env python

import sys
import os
import copy
import rospy
import argparse

import moveit_msgs.msg
import geometry_msgs.msg

from tf import transformations as tfs
from o2as_aruco_ros.msg import Corners
from std_srvs.srv import Trigger

from o2as_routines.base import O2ASBaseRoutines
from aist_routines.base import AISTBaseRoutines
from math import radians, degrees

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

    'ur5e': {
        'c_bot': [
            radians(-90), radians( 90), radians(0)],
        'd_bot': [
            radians(  0), radians( 90), radians(0)],
    },
}

######################################################################
#  global functions                                                  #
######################################################################
def format_pose(pose):
    rpy = map(
        degrees,
        tfs.euler_from_quaternion([
            pose.orientation.w, pose.orientation.x, pose.orientation.y,
            pose.orientation.z
        ]))
    return "[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]".format(
        pose.position.x, pose.position.y, pose.position.z, rpy[0], rpy[1],
        rpy[2])


######################################################################
#  class VisitRoutines                                               #
######################################################################
class VisitRoutines:
    """Wrapper of MoveGroupCommander specific for this script"""

    def __init__(self, routines, camera_name, robot_name, orientations):
        self.routines = routines

        cs = "/{}/".format(camera_name)
        self.start_acquisition = rospy.ServiceProxy(cs + "start_acquisition",
                                                    Trigger)
        self.stop_acquisition = rospy.ServiceProxy(cs + "stop_acquisition",
                                                   Trigger)

        # Initialize `moveit_commander`
        self.group_name = robot_name
        group = self.routines.groups[self.group_name]

        # Set `_ee_link` as end effector wrt `_base_link` of the robot
        group.set_pose_reference_frame("workspace_center")
        if robot_name == 'b_bot':
            group.set_end_effector_link(robot_name +
                                        "_single_suction_gripper_pad_link")
        else:
            group.set_end_effector_link(robot_name + "_robotiq_85_tip_link")

        # Logging
        print("==== Planning frame:       %s" % group.get_planning_frame())
        print("==== Pose reference frame: %s" %
              group.get_pose_reference_frame())
        print("==== End effector link:    %s" % group.get_end_effector_link())

        self.orientations = orientations

    def move(self, speed):
        self.start_acquisition()
        position = rospy.wait_for_message("/aruco_tracker/position",
                                          geometry_msgs.msg.Vector3Stamped, 10)
        self.stop_acquisition()

        group = self.routines.groups[self.group_name]
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.header.frame_id = group.get_pose_reference_frame()
        poseStamped.pose.position.x = position.vector.x
        poseStamped.pose.position.y = position.vector.y
        poseStamped.pose.position.z = position.vector.z + 0.05
        poseStamped.pose.orientation \
          = geometry_msgs.msg.Quaternion(
            *tfs.quaternion_from_euler(self.orientations[0],
                                       self.orientations[1],
                                       self.orientations[2]))
        [all_close, move_success] \
            = self.routines.go_to_pose_goal(
                  self.group_name, poseStamped, speed,
                  end_effector_link=group.get_end_effector_link(),
                  move_lin=False)
        rospy.sleep(1)

        poseStamped.pose.position.z = position.vector.z + 0.005
        print("     move to " + format_pose(poseStamped.pose))
        [all_close, move_success] \
            = self.routines.go_to_pose_goal(
                  self.group_name, poseStamped, speed,
                  end_effector_link=group.get_end_effector_link(),
                  move_lin=False)

        poseReached = self.routines.listener.transformPose(
            group.get_pose_reference_frame(), group.get_current_pose())
        print("  reached to " + format_pose(poseReached.pose))

    def go_home(self):
        self.routines.go_to_named_pose("home", self.group_name)

    def run(self, speed):
        self.stop_acquisition()
        self.go_home()

        while True:
            try:
                key = raw_input(">> ")
                if key == 'q':
                    break
                self.move(speed)
            except Exception as ex:
                self.stop_acquisition()
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

    if args.config == 'aist':
        base_routines = AISTBaseRoutines({args.robot_name})
    else:
        base_routines = O2ASBaseRoutines()
    camera_name = args.camera_name
    robot_name = args.robot_name

    assert (camera_name in {"a_phoxi_m_camera", "a_bot_camera"})
    assert (robot_name in {"a_bot", "b_bot", "c_bot"})

    routines = VisitRoutines(base_routines, camera_name, robot_name,
                             orientations[args.config][args.robot_name])
    speed = 0.05
    routines.run(speed)
