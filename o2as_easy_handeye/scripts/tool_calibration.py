#!/usr/bin/env python

import sys
import os
import copy
import rospy
import argparse
import moveit_msgs.msg

from geometry_msgs import msg as msg
from tf import TransformBroadcaster, TransformListener, \
               TransformerROS, transformations as tfs
from math import radians, degrees
from o2as_routines.base import O2ASBaseRoutines
from aist_routines.base import AISTBaseRoutines


refposes = {
  'a_bot':[ 0.10, -0.10, 0.20, radians(-90), radians( 90), radians(0)],
  # 'b_bot':[ 0.20,  0.10, 0.081, radians(  0), radians( 90), radians(-90)],
  'b_bot':[ 0.0,  0.0, 0.01, radians(  0), radians( 90), radians(-90)],
  'c_bot':[-0.30,  0.00, 0.35, radians(  0), radians( 90), radians(0)],
}


######################################################################
#  class ToolCalibrationRoutines                                     #
######################################################################
class ToolCalibrationRoutines:
  def __init__(self, routines, robot_name, speed):
    self.routines    = routines
    self.speed       = speed
    self.refpose     = refposes[robot_name]

    ## Initialize `moveit_commander`
    self.group_name = robot_name
    group = self.routines.groups[self.group_name]

    # Set `_ee_link` as end effector wrt `_base_link` of the robot
    group.set_pose_reference_frame('workspace_center')
    if robot_name == 'b_bot':
      gripper_base_link = robot_name + '_single_suction_gripper_base_link'
      gripper_tip_link  = robot_name + '_single_suction_gripper_pad_link'
    else:
      gripper_base_link = robot_name + '_gripper_base_link'
      gripper_tip_link  = robot_name + '_gripper_tip_link'
    self.gripper_base_link = gripper_base_link
    group.set_end_effector_link(gripper_tip_link)

    # Logging
    print("==== Planning frame:       %s" % group.get_planning_frame())
    print("==== Pose reference frame: %s" % group.get_pose_reference_frame())
    print("==== End effector link:    %s" % group.get_end_effector_link())

    self.listener = TransformListener()
    now = rospy.Time.now()
    self.listener.waitForTransform(gripper_base_link, gripper_tip_link,
                                   now, rospy.Duration(10))
    self.D0 = self.listener.fromTranslationRotation(
                *self.listener.lookupTransform(gripper_base_link,
                                               gripper_tip_link, now))
    self.pitch = 0.0
    self.yaw   = 0.0


  def go_home(self):
    self.routines.go_to_named_pose('home', self.group_name)

  def correct_end_effector_link(self):
    D = tfs.concatenate_matrices(
          self.listener.fromTranslationRotation(
            (0, 0, 0), tfs.quaternion_from_euler(0, self.pitch, self.yaw)),
          self.D0)
    group = self.routines.groups[self.group_name]
    print('  trns = {}, rot = {}'.format(tfs.translation_from_matrix(D),
                                         tfs.quaternion_from_matrix(D)))
    rate = rospy.Rate(10.0)
    self.broadcaster.sendTransform(
      tfs.translation_from_matrix(D), tfs.quaternion_from_matrix(D),
      rospy.Time.now(),
      group.get_end_effector_link() + '_corrected', self.gripper_base_link)
    rate.sleep()


  def print_propmt():


  def move(self, pose):
    R = self.listener.fromTranslationRotation(
            (0, 0, 0), tfs.quaternion_from_euler(0, self.pitch, self.yaw))
    T = tfs.concatenate_matrices(
          self.listener.fromTranslationRotation((pose[0], pose[1], pose[2]),
                                                tfs.quaternion_from_euler(
                                                  pose[3], pose[4], pose[5])),
          tfs.inverse_matrix(self.D0),
          tfs.inverse_matrix(R),
          self.D0)
    group = self.routines.groups[self.group_name]
    poseStamped                 = msg.PoseStamped()
    poseStamped.header.frame_id = group.get_pose_reference_frame()
    poseStamped.pose = msg.Pose(msg.Point(*tfs.translation_from_matrix(T)),
                                msg.Quaternion(*tfs.quaternion_from_matrix(T)))
    print("*move to*\n{}".format(poseStamped.pose.position))
    [all_close, move_success] \
      = self.routines.go_to_pose_goal(
                            self.group_name, poseStamped, self.speed,
                            end_effector_link=group.get_end_effector_link(),
                            move_lin=False)
    rospy.sleep(1)
    poseReached = self.listener.transformPose(group.get_pose_reference_frame(),
                                              group.get_current_pose())
    print("*reached*\n{}\n".format(poseReached.pose.position))

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


  def print_tip_link(self):
    R   = self.listener.fromTranslationRotation(
            (0, 0, 0), tfs.quaternion_from_euler(0, self.pitch, self.yaw))
    D   = tfs.concatenate_matrices(R, self.D0)
    xyz = tfs.translation_from_matrix(D)
    q   = tfs.quaternion_from_matrix(D)
    rpy = map(degrees, tfs.euler_from_quaternion(q))
    print('<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'.format(xyz, rpy))


  def run(self):
    # Reset pose
    self.go_home()

    self.move(self.refpose)

    axis = 'Pitch'

    while True:
      prompt = axis + '[p=' + str(degrees(self.pitch)) + \
                      ',y=' + str(degrees(self.yaw)) + '] >> '
      key = raw_input(prompt)
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
      elif key == 'X':
        axis = 'X    '
      elif key == 'Y':
        axis = 'Y    '
      elif key == 'Z':
        axis = 'Z    '
      elif key == 'P':
        axis = 'Pitch'
      elif key == 'Y':
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
          self.yaw   += radians(0.5)
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
          self.yaw   -= radians(0.5)
        self.move(self.refpose)
      elif key == 'd':
        self.print_tip_link()
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
          self.yaw   = radians(float(key))
        self.move(self.refpose)

    # Reset pose
    self.go_home()
    self.print_tip_link()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
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

  try:
    assert(args.robot_name  in {'a_bot', 'b_bot', 'c_bot'})

    if args.config == 'aist':
      base_routines = AISTBaseRoutines()
    else:
      base_routines = O2ASBaseRoutines()

    speed    = 1
    routines = ToolCalibrationRoutines(base_routines, args.robot_name, speed)

    routines.run()

  except Exception as ex:
    print(ex.message)
