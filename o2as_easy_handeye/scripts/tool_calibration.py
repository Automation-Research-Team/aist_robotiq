#!/usr/bin/env python

import sys
import os
import copy
import rospy
import argparse
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from math import radians, degrees
from o2as_routines.base import O2ASBaseRoutines
from aist_routines.base import AISTBaseRoutines


refposes = {
  'a_bot':[ 0.10, -0.10, 0.20, radians(-90), radians( 90), radians(0)],
  # 'b_bot':[ 0.20,  0.10, 0.081, radians(  0), radians( 90), radians(-90)],
  'b_bot':[ 0.0,  0.0, 0.005, radians(  0), radians( 90), radians(-90)],
  'c_bot':[-0.30,  0.00, 0.35, radians(  0), radians( 90), radians(0)],
}


######################################################################
#  class ToolCalibrationRoutines                                     #
######################################################################
class ToolCalibrationRoutines:
  def __init__(self, routines, robot_name, speed, sleep_time):
    self.routines    = routines
    self.speed       = speed
    self.sleep_time  = sleep_time
    self.refpose     = refposes[robot_name]

    ## Initialize `moveit_commander`
    self.group_name = robot_name
    group = self.routines.groups[self.group_name]

    # Set `_ee_link` as end effector wrt `_base_link` of the robot
    group.set_pose_reference_frame("workspace_center")
    if robot_name == 'b_bot':
      self.gripper_base_link = robot_name + '_single_suction_gripper_base_link'
      self.gripper_tip_link  = robot_name + '_single_suction_gripper_pad_link'
    else:
      self.gripper_base_link = robot_name + '_gripper_base_link'
      self.gripper_tip_link  = robot_name + '_gripper_tip_link'
    group.set_end_effector_link(self.gripper_tip_link)

    # Trajectory publisher
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size=20
    )

    # Logging
    print("============ Reference frame: %s" % group.get_planning_frame())
    print("============ End effector: %s"    % group.get_end_effector_link())

    self.listener    = tf.TransformListener()
    self.broadcaster = tf.TransformBroadcaster()


  def go_home(self):
    self.routines.go_to_named_pose("home", self.group_name)


  def move(self, pose):
    print("move to {}".format(pose))
    group = self.routines.groups[self.group_name]
    poseStamped                 = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id = group.get_pose_reference_frame()
    poseStamped.pose.position.x = pose[0]
    poseStamped.pose.position.y = pose[1]
    poseStamped.pose.position.z = pose[2]
    poseStamped.pose.orientation \
      = geometry_msgs.msg.Quaternion(
        *tf.transformations.quaternion_from_euler(
          pose[3], pose[4], pose[5]))
    [all_close, move_success] \
      = self.routines.go_to_pose_goal(
                            self.group_name, poseStamped, self.speed,
                            end_effector_link=group.get_end_effector_link(),
                            move_lin=False)
    return move_success


  def rolling_motion(self):
    pose = copy.deepcopy(self.refpose)
    for i in range(5):
      pose[3] += radians(30)
      self.move(pose)
    for i in range(10):
      pose[3] -= radians(30)
      self.move(pose)
    for i in range(5):
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


  def adjust_tip_link(self, dp, dy):
    now = rospy.Time.now()
    self.listener.waitForTransform(self.gripper_tip_link,
                                   self.gripper_base_link,
                                   now, rospy.Duration(10))
    xyz, q = self.listener.lookupTransform(self.gripper_tip_link,
                                           self.gripper_base_link, now)
    rpy = tf.transformations.euler_from_quaternion(q)
    q = tf.transformations.quaternion_from_euler(rpy[0],
                                                 rpy[1] + dp, rpy[2] + dy)
    self.broadcaster.sendTransform(xyz, q, now,
                                   self.gripper_tip_link,
                                   self.gripper_base_link)


  def print_tip_link(self):
    now = rospy.Time.now()
    self.listener.waitForTransform(self.gripper_tip_link,
                                   self.gripper_base_link,
                                   now, rospy.Duration(10))
    xyz, q = self.listener.lookupTransform(self.gripper_tip_link,
                                           self.gripper_base_link, now)
    rpy = map(degrees, tf.transformations.euler_from_quaternion(q))
    print "<origin xyz=\"{0[0]} {0[1]} {0[2]}\" rpy=\"${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}\"/>".format(xyz, rpy)


  def run(self):
    # Reset pose
    self.go_home()

    self.move(self.refpose)

    while True:
      try:
        key = raw_input(">> ")
        if key == 'q':
          break
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
        elif key == '+':
          self.adjust_tip_link(radians( 5), radians( 0.0))
        elif key == '-':
          self.adjust_tip_link(radians(-5), radians( 0.0))
        elif key == '>':
          self.adjust_tip_link(radians( 0.0), radians( 5))
        elif key == '<':
          self.adjust_tip_link(radians( 0.0), radians(-5))
        elif key == 'd':
          self.print_tip_link()
      except Exception as ex:
        print ex.message
      except rospy.ROSInterruptException:
        return
      except KeyboardInterrupt:
        break

    # Reset pose
    self.go_home()
    self.print_tip_link()


######################################################################
#  global functions                                                  #
######################################################################
def main():
  try:
    parser = argparse.ArgumentParser(description='Do hand-eye calibration')
    parser.add_argument('-C', '--config',
                        action='store', nargs='?',
                        default='aist', type=str, choices=None,
                        help='configuration name', metavar=None)
    parser.add_argument('-r', '--robot_name',
                        action='store', nargs='?',
                        default='b_bot', type=str, choices=None,
                        help='robot name', metavar=None)

    args = parser.parse_args()
    print(args)

    if args.config == 'aist':
      base_routines = AISTBaseRoutines()
    else:
      base_routines = O2ASBaseRoutines()
    robot_name = args.robot_name

    assert(robot_name  in {"a_bot", "b_bot", "c_bot"})

    speed      = 1
    sleep_time = 1
    routines   = ToolCalibrationRoutines(base_routines, robot_name,
                                         speed, sleep_time)

    routines.run()

  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()
