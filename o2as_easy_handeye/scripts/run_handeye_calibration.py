#!/usr/bin/env python

import sys
import os
import copy
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions

from math import radians, degrees

from std_msgs.msg import String
from std_srvs.srv import Empty
from std_srvs.srv import Trigger
from o2as_phoxi_camera.srv import GetFrame
from easy_handeye.srv import TakeSample, RemoveSample, ComputeCalibration

from o2as_routines.base import O2ASBaseRoutines

# Poses taken during handeye calibration
# TODO: These poses are specific for `d_bot` camera, need to modify for Phoxi
poses = {
  'a_phoxi_m_camera': {
    'a_bot': [
      [0.245, 0.3, 0.30, radians(  0), radians(  0), radians(90)],
      [0.245, 0.3, 0.30, radians(  0), radians( 30), radians(90)],
      [0.245, 0.3, 0.30, radians( 30), radians( 30), radians(90)],
      [0.245, 0.3, 0.30, radians( 30), radians(  0), radians(90)],
      [0.245, 0.3, 0.30, radians(-30), radians(  0), radians(90)],
      [0.245, 0.3, 0.30, radians(-30), radians( 30), radians(90)],
    ],

    'b_bot': [
      [-0.32, 0.30, 0.15, radians( 30), radians( 25), radians(180)],
      [-0.32, 0.45, 0.15, radians( 30), radians( 25), radians(180)],
      [-0.35, 0.60, 0.15, radians(  0), radians( 25), radians(180)],

      [-0.35, 0.50, 0.25, radians(  0), radians( 25), radians(180)],
      [-0.32, 0.40, 0.25, radians( 30), radians( 25), radians(180)],
      [-0.32, 0.30, 0.25, radians( 30), radians( 25), radians(180)],

      [-0.20, 0.30, 0.30, radians( 30), radians( 25), radians(180)],
      [-0.20, 0.40, 0.30, radians( 30), radians( 25), radians(180)],
      [-0.20, 0.50, 0.30, radians(  0), radians( 25), radians(180)],

      [-0.15, 0.60, 0.15, radians(  0), radians( 25), radians(180)],
      [-0.15, 0.45, 0.15, radians( 30), radians( 25), radians(180)],
      [-0.15, 0.30, 0.15, radians( 30), radians( 25), radians(180)],
    ],

    'c_bot': [
      [-0.110,  0.433,  0.386, -0.900,  0.000,  0.000],
      [-0.110,  0.415,  0.351, -0.570,  0.000,  0.000],
      [-0.143,  0.424,  0.365, -0.699,  0.559, -0.428]
    ]
  }
}


def get_service_proxy(service_name, camera_name, robot_name):
  """Return ROS service proxy"""
  cs = "/{}/".format(camera_name)
  ns = "/o2as_easy_handeye_{}_eye_on_base/".format(robot_name)

  if service_name is "trigger_frame":
    service_type      = Trigger
    service_name_full = cs + "trigger_frame"
  elif service_name is "get_frame":
    service_type      = GetFrame
    service_name_full = cs + "get_frame"
  elif service_name is "take_sample":
    service_type      = TakeSample
    service_name_full = ns + "take_sample"
  elif service_name is "get_sample_list":
    service_type      = TakeSample
    service_name_full = ns + "get_sample_list"
  elif service_name is "remove_sample":
    service_type      = RemoveSample
    service_name_full = ns + "remove_sample"
  elif service_name is "compute_calibration":
    service_type      = ComputeCalibration
    service_name_full = ns + "compute_calibration"
  elif service_name is "save_calibration":
    service_type      = Empty
    service_name_full = ns + "save_calibration"
  else:
    raise NameError("Service name {} does not exist".format(service_name))

  return rospy.ServiceProxy(service_name_full, service_type)


class MoveGroupCommander(object):
  """Wrapper of MoveGroupCommander specific for this script"""
  def __init__(self, camera_name, robot_name):
    self.trigger_frame       = get_service_proxy("trigger_frame",
                                                 camera_name, robot_name)
    self.get_frame           = get_service_proxy("get_frame",
                                                 camera_name, robot_name)
    self.take_sample         = get_service_proxy("take_sample",
                                                 camera_name, robot_name)
    self.get_sample_list     = get_service_proxy("get_sample_list",
                                                 camera_name, robot_name)
    self.remove_sample       = get_service_proxy("remove_sample",
                                                 camera_name, robot_name)
    self.compute_calibration = get_service_proxy("compute_calibration",
                                                 camera_name, robot_name)
    self.save_calibration    = get_service_proxy("save_calibration",
                                                 camera_name, robot_name)

    ## Initialize `moveit_commander`
    self.baseRoutines = O2ASBaseRoutines()
    self.robot_name   = robot_name
    group = self.baseRoutines.groups[robot_name]

    # Set `_ee_link` as end effector wrt `_base_link` of the robot
    group.set_pose_reference_frame(robot_name + "_base_link")
    # group.set_end_effector_link(robot_name    + "_ee_link")
    group.set_end_effector_link(robot_name    + "_ar_marker")

    # Trajectory publisher
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size=20
    )

    # Logging
    print("============ Reference frame: %s" % group.get_planning_frame())
    print("============ End effector: %s"    % group.get_end_effector_link())
    
  def visit_subposes(self, keypose, speed, sleep_time):
    pose = keypose
    for i in range(3):
      print("\n--- SubLoop [{}/5]: Try! ---".format(i+1))
      self.move(pose, speed)
      print("--- Subloop [{}/5]: Completed. ---".format(i+1))
      self.trigger_frame()
      self.get_frame(0, True)
      rospy.sleep(sleep_time)
      pose[3] -= radians(30)

    pose[3]  = radians(0)
    pose[4] += radians(15)

    for i in range(2):
      print("\n--- Subloop [{}/5]: Try! ---".format(i+4))
      self.move(pose, speed)
      print("--- Subloop [{}/5]: Completed. ---".format(i+4))
      self.trigger_frame()
      self.get_frame(0, True)
      rospy.sleep(sleep_time)
      pose[4] -= radians(30)

    
  def move(self, pose, speed):
    """Move the end effector"""
    # TODO: check type of `pose`
    print("move to {}".format(pose))
    poseStamped                 = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id = self.robot_name + "_base_link"
    poseStamped.pose.position.x = pose[0]
    poseStamped.pose.position.y = pose[1]
    poseStamped.pose.position.z = pose[2]
    poseStamped.pose.orientation \
      = geometry_msgs.msg.Quaternion(
        *tf_conversions.transformations.quaternion_from_euler(
          pose[3], pose[4], pose[5]))
    self.baseRoutines.go_to_pose_goal(self.robot_name, poseStamped, speed,
                                      move_lin=False)

  def take_sample(self):
    self.trigger_frame()
    self.get_frame(0, True)
    self.take_sample()

  def go_home(self):
    self.baseRoutines.go_to_named_pose("home", self.robot_name)

    
def run_calibration(camera_name, robot_name, speed):
  """Run handeye calibration for the specified robot (e.g., "b_bot")"""
  # Initialize move group and service proxies
  mg = MoveGroupCommander(camera_name, robot_name)

  print("=== Calibration started for {} ===".format(robot_name))

  # Clear samples in the buffer if exist
  n_samples = len(get_sample_list().samples.hand_world_samples.transforms)
  if 0 < n_samples:
    for _ in range(n_samples):
      remove_sample(0)

  # Reset pose
  mg.go_home()

  # Collect samples over pre-defined poses
  pose_list = poses[camera_name][robot_name]
  for i, pose in enumerate(pose_list):
    print("\n*** Loop [{}/{}]: Try! ***".format(i+1, len(pose_list)))
    mg.move(pose, speed)
    print("*** Loop [{}/{}]: Completed. ***".format(i+1, len(pose_list)))
    mg.take_sample()
    rospy.sleep(3)  # Sleep for 1 seconds
    sample_list = mg.get_sample_list()
    n1 = len(sample_list.samples.hand_world_samples.transforms)
    n2 = len(sample_list.samples.camera_marker_samples.transforms)
    print("  took {} hand_world samples and {} camera_marker " +
          "samples").format(n1, n2)

  # Compute and save calibration
  mg.compute_calibration()
  mg.save_calibration()

  # Reset pose
  mg.go_home()

  print("=== Calibration completed for {} ===".format(robot_name))

  
def visit_calibration_points(camera_name, robot_name, speed, sleep_time):
  """Run handeye calibration for the specified robot (e.g., "b_bot")"""
  # Initialize move group and service proxies
  mg = MoveGroupCommander(camera_name, robot_name)

  print("=== Visiting started for {} ===".format(robot_name))

  # Reset pose
  mg.go_home()

  # Move to each calibration point
  pose_list = poses[camera_name][robot_name]
  for i, pose in enumerate(pose_list):
    print("\n*** Loop [{}/{}]: Try! ***".format(i+1, len(pose_list)))
    mg.visit_subposes(pose, speed, sleep_time)
    print("*** Loop [{}/{}]: Completed. ***".format(i+1, len(pose_list)))

  # Reset pose
  mg.go_home()
  print("=== Visiting completed for {} ===".format(robot_name))

  
def main():
  try:
    camera_name = sys.argv[1]
    robot_name  = sys.argv[2]
    assert(camera_name in {"a_phoxi_m_camera", "c_bot_camera"})
    assert(robot_name  in {"a_bot", "b_bot", "c_bot"})
    speed = 0.1;

    if (os.path.basename(sys.argv[0]) == "run_handeye_calibration.py"):
      run_calibration(camera_name, robot_name, speed)
    else:
      visit_calibration_points(camera_name, robot_name, speed, 1)

  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()
