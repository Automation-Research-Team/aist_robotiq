#!/usr/bin/env python

import sys
import os
import copy
import rospy
import argparse
import moveit_msgs.msg
import geometry_msgs.msg

from math import radians, degrees
from tf   import transformations as tfs

from std_msgs.msg import String
from std_srvs.srv import Empty
from std_srvs.srv import Trigger

from aist_routines.base import AISTBaseRoutines


initposes = {
  'a_bot':[ 0.10, -0.10, 0.20, radians(-90), radians( 90), radians(0)],
  'b_bot':[ 0.10,  0.10, 0.35, radians(  0), radians( 90), radians(0)],
  'c_bot':[-0.30,  0.00, 0.35, radians(  0), radians( 90), radians(0)],
  'd_bot':[-0.30,  0.00, 0.35, radians(  0), radians( 90), radians(0)],
}

termposes = {
  'a_bot':[ 0.10, -0.10, 0.20,  radians(-90), radians( 90), radians(0)],
  'b_bot':[ 0.1,   0.06, 0.535, radians(  0), radians( 90), radians(0)],
  'c_bot':[-0.30,  0.00, 0.35,  radians(  0), radians( 90), radians(0)],
  'd_bot':[-0.30,  0.00, 0.35,  radians(  0), radians( 90), radians(0)],
}

# Poses taken during handeye calibration
aist_keyposes = {
  'a_bot': [
    [0.15, -0.10, 0.13, radians( 30), radians( 25), radians(0)],
    [0.15,  0.00, 0.13, radians( 30), radians( 25), radians(0)],
    [0.15,  0.10, 0.13, radians( 30), radians( 25), radians(0)],

    [0.15,  0.10, 0.23, radians( 30), radians( 25), radians(0)],
    [0.15,  0.00, 0.23, radians( 30), radians( 25), radians(0)],
    [0.15, -0.10, 0.23, radians( 30), radians( 25), radians(0)],
  ],

  'b_bot': [
    [0.15,  0.20, 0.16, radians( 30), radians( 25), radians(0)],
    [0.15,  0.10, 0.16, radians( 30), radians( 25), radians(0)],
    [0.15, -0.00, 0.16, radians( 30), radians( 25), radians(0)],

    [0.15, -0.00, 0.25, radians( 30), radians( 25), radians(0)],
    [0.15,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
    [0.15,  0.20, 0.25, radians( 30), radians( 25), radians(0)],

    # [0.40,  0.15, 0.15, radians( 30), radians( 25), radians(0)],
    # [0.40,  0.00, 0.15, radians( 30), radians( 25), radians(0)],
    # [0.40, -0.15, 0.15, radians(  0), radians( 25), radians(0)],

    # [0.35, -0.10, 0.10, radians(  0), radians( 25), radians(0)],
    # [0.35,  0.05, 0.10, radians( 30), radians( 25), radians(0)],
    # [0.35,  0.20, 0.10, radians( 30), radians( 25), radians(0)],
  ],

  'c_bot': [
    [ 0.3,  0.10, 0.25, radians( 90), radians( 90), radians(0)],
    [ 0.3,  0.00, 0.25, radians( 90), radians( 90), radians(0)],
    [ 0.3, -0.10, 0.25, radians( 90), radians( 90), radians(0)],

    [ 0.3, -0.10, 0.40, radians( 90), radians( 90), radians(0)],
    [ 0.3,  0.00, 0.40, radians( 90), radians( 90), radians(0)],
    [ 0.3,  0.10, 0.40, radians( 90), radians( 90), radians(0)],

    [-0.3,  0.10, 0.40, radians( 90), radians( 90), radians(0)],
    [-0.3, -0.00, 0.40, radians( 90), radians( 90), radians(0)],
    [-0.3, -0.10, 0.40, radians( 90), radians( 90), radians(0)],

    [-0.3, -0.10, 0.25, radians( 90), radians( 90), radians(0)],
    [-0.3, -0.00, 0.25, radians( 90), radians( 90), radians(0)],
    [-0.3,  0.10, 0.25, radians( 90), radians( 90), radians(0)],
  ],

  'd_bot': [
    [0.15,  0.20, 0.16, radians( 30), radians( 25), radians(0)],
    [0.15,  0.10, 0.16, radians( 30), radians( 25), radians(0)],
    [0.15, -0.00, 0.16, radians( 30), radians( 25), radians(0)],

    [0.15, -0.00, 0.25, radians( 30), radians( 25), radians(0)],
    [0.15,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
    [0.15,  0.20, 0.25, radians( 30), radians( 25), radians(0)],
  ]
}

######################################################################
#  global functions                                                  #
######################################################################
def format_pose(pose):
  rpy = map(degrees, tfs.euler_from_quaternion(
                        [pose.orientation.w, pose.orientation.x,
                         pose.orientation.y, pose.orientation.z]))
  return "[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]".format(
    pose.position.x, pose.position.y, pose.position.z, rpy[0], rpy[1], rpy[2])


######################################################################
#  class MotionRoutines                                              #
######################################################################
class MotionRoutines:
  def __init__(self, routines, robot_name, speed, sleep_time):
    self.routines    = routines
    self.speed       = speed
    self.sleep_time  = sleep_time

    ## Initialize `moveit_commander`
    self.group_name = robot_name
    group = self.routines.groups[self.group_name]

    # Set `_ee_link` as end effector wrt `_base_link` of the robot
    #group.set_planning_frame("workspace_center")
    group.set_pose_reference_frame("workspace_center")
    group.set_end_effector_link(robot_name + "_ee_link")

    # Logging
    print("==== Planning frame:       %s" % group.get_planning_frame())
    print("==== Pose reference frame: %s" % group.get_pose_reference_frame())
    print("==== End effector link:    %s" % group.get_end_effector_link())


  def go_home(self):
    self.routines.go_to_named_pose("home", self.group_name)


  def move(self, pose):
    group = self.routines.groups[self.group_name]
    poseStamped                  = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id  = group.get_pose_reference_frame()
    poseStamped.pose.position.x  = pose[0]
    poseStamped.pose.position.y  = pose[1]
    poseStamped.pose.position.z  = pose[2]
    poseStamped.pose.orientation = geometry_msgs.msg.Quaternion(
                                     *tfs.quaternion_from_euler(
                                       pose[3], pose[4], pose[5]))
    print("     move to " + format_pose(poseStamped.pose))
    [all_close, move_success] \
      = self.routines.go_to_pose_goal(
                            self.group_name, poseStamped, self.speed,
                            end_effector_link=group.get_end_effector_link(),
                            move_lin=False)
    poseReached = self.routines.listener.transformPose(
                        group.get_pose_reference_frame(),
                        group.get_current_pose())
    print("  reached to " + format_pose(poseReached.pose))
    rospy.sleep(self.sleep_time)
    return move_success


  def run(self, initpose, keyposes, termpose):
    # Reset pose
    self.go_home()
    # self.move(initpose)

    # Collect samples over pre-defined poses
    for i, keypose in enumerate(keyposes):
      print("\n*** Keypose [{}/{}]: Try! ***".format(i+1, len(keyposes)))
      self.move(keypose)
      print("*** Keypose [{}/{}]: Completed. ***".format(i+1, len(keyposes)))

    # Reset pose
    # self.move(termpose)
    self.go_home()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='Do hand-eye calibration')
  parser.add_argument('-r', '--robot_name',
                      action='store', nargs='?',
                      default='b_bot', type=str, choices=None,
                      help='robot name', metavar=None)

  args = parser.parse_args()

  try:
    robot_name    = args.robot_name
    base_routines = AISTBaseRoutines({robot_name})

    assert(robot_name in {"a_bot", "b_bot", "c_bot", "d_bot"})

    speed      = 1
    sleep_time = 0
    routines   = MotionRoutines(base_routines, robot_name, speed, sleep_time)

    print("=== Begin for {} ===".format(robot_name))
    routines.run(initposes[robot_name],
                 aist_keyposes[robot_name],
                 termposes[robot_name])
    print("=== Completed for {} ===".format(robot_name))

  except Exception as ex:
    print(ex.message)
