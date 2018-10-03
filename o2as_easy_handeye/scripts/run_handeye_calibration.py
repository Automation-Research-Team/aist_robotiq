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
keyposes = {
  'a_phoxi_m_camera': {
    'a_bot': [
#      [0.15, -0.30, 0.25, radians( 30), radians( 25), radians(0)],
      [0.15, -0.10, 0.20, radians(-90), radians( 25), radians(0)],
      [0.15,  0.00, 0.20, radians(-60), radians( 25), radians(0)],
      [0.15,  0.10, 0.20, radians(-60), radians( 25), radians(0)],

      [0.20,  0.10, 0.30, radians(-60), radians( 25), radians(0)],
      [0.20,  0.00, 0.30, radians(-60), radians( 25), radians(0)],
      [0.20, -0.10, 0.30, radians(-60), radians( 25), radians(0)],
    ],

    'b_bot': [
      [0.32,  0.15, 0.10, radians( 30), radians( 25), radians(0)],
      [0.32,  0.00, 0.10, radians( 30), radians( 25), radians(0)],
      [0.35, -0.10, 0.10, radians(  0), radians( 25), radians(0)],

      [0.35,  0.00, 0.25, radians(  0), radians( 25), radians(0)],
      [0.32,  0.10, 0.25, radians( 30), radians( 25), radians(0)],
      [0.32,  0.20, 0.25, radians( 30), radians( 25), radians(0)],

      [0.20,  0.20, 0.30, radians( 30), radians( 25), radians(0)],
      [0.20,  0.10, 0.30, radians( 30), radians( 25), radians(0)],
      [0.20,  0.00, 0.30, radians(  0), radians( 25), radians(0)],

      [0.15, -0.10, 0.15, radians(  0), radians( 25), radians(0)],
      [0.15,  0.05, 0.15, radians( 30), radians( 25), radians(0)],
      [0.15,  0.20, 0.15, radians( 30), radians( 25), radians(0)],
    ],

    'c_bot': [
      [0.05, -0.05, 0.15, radians(  0), radians( 25), radians(0)],
      [0.05,  0.05, 0.15, radians( 30), radians( 25), radians(0)],
      [0.05,  0.15, 0.15, radians( 30), radians( 25), radians(0)],

      [0.15,  0.15, 0.15, radians( 30), radians( 25), radians(0)],
      [0.15,  0.05, 0.15, radians( 30), radians( 25), radians(0)],
      [0.15, -0.05, 0.15, radians(  0), radians( 25), radians(0)],

      [0.15, -0.05, 0.25, radians(  0), radians( 25), radians(0)],
      [0.15,  0.05, 0.25, radians( 30), radians( 25), radians(0)],
      [0.15,  0.15, 0.25, radians( 30), radians( 25), radians(0)],

      [0.10,  0.15, 0.25, radians( 30), radians( 25), radians(0)],
      [0.10,  0.05, 0.25, radians( 30), radians( 25), radians(0)],
      [0.10, -0.05, 0.25, radians(  0), radians( 25), radians(0)],
    ]
  }
}


def get_service_proxy(service_name, camera_name, robot_name):
  """Return ROS service proxy"""
  cs = "/{}/".format(camera_name)
  ns = "/o2as_easy_handeye_{}_eye_on_base/".format(robot_name)

  if service_name is "flush_buffer":
    service_type      = Trigger
    service_name_full = cs + "flush_buffer"
  elif service_name is "trigger_frame":
    service_type      = Trigger
    service_name_full = cs + "trigger_frame"
  elif service_name is "get_frame":
    service_type      = GetFrame
    service_name_full = cs + "get_frame"
  elif service_name is "start_acquisition":
    service_type      = Trigger
    service_name_full = cs + "start_acquisition"
  elif service_name is "stop_acquisition":
    service_type      = Trigger
    service_name_full = cs + "stop_acquisition"
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


######################################################################
#  class CalibrationCommander                                        #
######################################################################
class HandEyeCalibrationRoutines(O2ASBaseRoutines):
  """Wrapper of MoveGroupCommander specific for this script"""
  def __init__(self, camera_name, robot_name, needs_trigger, needs_calib):
    super(HandEyeCalibrationRoutines, self).__init__()
    
    self.needs_trigger = needs_trigger
    self.needs_calib   = needs_calib

    if needs_trigger:
      self.flush_buffer      = get_service_proxy("flush_buffer",
                                                 camera_name, robot_name)
      self.trigger_frame     = get_service_proxy("trigger_frame",
                                                 camera_name, robot_name)
      self.get_frame         = get_service_proxy("get_frame",
                                                 camera_name, robot_name)
      self.start_acquisition = get_service_proxy("start_acquisition",
                                                 camera_name, robot_name)
      self.stop_acquisition  = get_service_proxy("stop_acquisition",
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
    self.robot_name   = robot_name
    group = self.groups[robot_name]

    # Set `_ee_link` as end effector wrt `_base_link` of the robot
    #group.set_pose_reference_frame(robot_name + "_base_link")
    group.set_pose_reference_frame("workspace_center")
    group.set_end_effector_link(robot_name    + "_ee_link")
    #group.set_end_effector_link(robot_name + "_ar_marker")

    # Trajectory publisher
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size=20
    )

    # Logging
    print("============ Reference frame: %s" % group.get_planning_frame())
    print("============ End effector: %s"    % group.get_end_effector_link())

    
  def move_to_subposes(self, pose, speed, sleep_time):
    roll = pose[3]
    for i in range(3):
      print("\n--- Subpose [{}/5]: Try! ---".format(i+1))
      if self.move(pose, speed):
        rospy.sleep(sleep_time)
        print("--- Subpose [{}/5]: Succeeded. ---".format(i+1))
      else:
        print("--- Subpose [{}/5]: Failed. ---".format(i+1))
      pose[3] -= radians(30)

    pose[3]  = roll - radians(30)
    pose[4] += radians(15)

    for i in range(2):
      print("\n--- Subpose [{}/5]: Try! ---".format(i+4))
      if self.move(pose, speed):
        rospy.sleep(sleep_time)
        print("--- Subpose [{}/5]: Succeeded. ---".format(i+4))
      else:
        print("--- Subpose [{}/5]: Failed. ---".format(i+4))
      pose[4] -= radians(30)

    
  def move(self, pose, speed):
    """Move the end effector"""
    # TODO: check type of `pose`
    print("move to {}".format(pose))
    poseStamped                 = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id = "workspace_center"
    poseStamped.pose.position.x = pose[0]
    poseStamped.pose.position.y = pose[1]
    poseStamped.pose.position.z = pose[2]
    poseStamped.pose.orientation \
      = geometry_msgs.msg.Quaternion(
        *tf_conversions.transformations.quaternion_from_euler(
          pose[3], pose[4], pose[5]))
    [all_close, move_success] = self.go_to_pose_goal(self.robot_name,
                                                     poseStamped, speed,
                                                     move_lin=False)
    if move_success:
      if self.needs_trigger:
        self.start_acquisition()
        rospy.sleep(1)
        # self.trigger_frame()
        # self.get_frame(0, True)

      if self.needs_calib:
        try:
          self.take_sample()
          sample_list = self.get_sample_list()
          n1 = len(sample_list.samples.hand_world_samples.transforms)
          n2 = len(sample_list.samples.camera_marker_samples.transforms)
          print("  took {} hand-world samples and {} camera-marker samples").format(n1, n2)
        except rospy.ServiceException as e:
          print "Service call failed: %s"%e

      if self.needs_trigger:
        self.stop_acquisition()

    return move_success
        

  def go_home(self):
    self.go_to_named_pose("home", self.robot_name)

  def run(self, keyposes, speed, sleep_time):
    """Run handeye calibration for the specified robot (e.g., "b_bot")"""
    # Clear samples in the buffer if exist
    if self.needs_trigger:
      self.stop_acquisition()
    
    if self.needs_calib:
      n_samples = len(self.get_sample_list().samples.hand_world_samples.transforms)
      if 0 < n_samples:
        for _ in range(n_samples):
          self.remove_sample(0)

    # Reset pose
    self.go_home()

    #self.flush_buffer()
  
    # Collect samples over pre-defined poses
    for i, keypose in enumerate(keyposes):
      print("\n*** Keypose [{}/{}]: Try! ***".format(i+1, len(keyposes)))
      self.move_to_subposes(keypose, speed, sleep_time)
      print("*** Keypose [{}/{}]: Completed. ***".format(i+1, len(keyposes)))

    if self.needs_calib:
      # Compute and save calibration
      self.compute_calibration()
      self.save_calibration()

    # Reset pose
    self.go_home()

    
######################################################################
#  global functions                                                  #
######################################################################
  
def main():
  try:
    camera_name   = sys.argv[1]
    robot_name    = sys.argv[2]
    needs_trigger = (True if (sys.argv[3] == "trigger") else False)
    needs_calib   = (True if (os.path.basename(sys.argv[0]) == "run_handeye_calibration.py") else False)
    
    assert(camera_name in {"a_phoxi_m_camera", "c_bot_camera"})
    assert(robot_name  in {"a_bot", "b_bot", "c_bot"})

    routines = HandEyeCalibrationRoutines(camera_name, robot_name,
                                          needs_trigger, needs_calib)

    print("=== Calibration started for {} + {} ===".format(camera_name,
                                                           robot_name))
    speed      = 0.1
    sleep_time = 1
    routines.run(keyposes[camera_name][robot_name], speed, sleep_time)
    print("=== Calibration completed for {} + {} ===".format(camera_name,
                                                             robot_name))
    

  except rospy.ROSInterruptException:
    return

  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  main()
