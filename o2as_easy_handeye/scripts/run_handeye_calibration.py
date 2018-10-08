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

import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError

# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Poses taken during handeye calibration
# TODO: These poses are specific for `d_bot` camera, need to modify for Phoxi
keyposes = {
  'a_phoxi_m_camera': {
    'a_bot': [
#      [0.15, -0.30, 0.25, radians( 30), radians( 25), radians(0)],
      [0.38, -0.10, 0.10, radians(-90), radians( 25), radians(0)],
      [0.38,  0.00, 0.10, radians(-60), radians( 25), radians(0)],
      [0.38,  0.10, 0.10, radians(-60), radians( 25), radians(0)],

      [0.30,  0.10, 0.20, radians(-60), radians( 25), radians(0)],
      [0.30,  0.00, 0.20, radians(-60), radians( 25), radians(0)],
      [0.30, -0.10, 0.20, radians(-60), radians( 25), radians(0)],
    ],

    'b_bot': [
      [0.38,  0.15, 0.10, radians( 30), radians( 25), radians(0)],
      [0.38,  0.00, 0.10, radians( 30), radians( 25), radians(0)],
      [0.38, -0.15, 0.10, radians(  0), radians( 25), radians(0)],

      [0.32, -0.10, 0.20, radians( 30), radians( 25), radians(0)],
      [0.32,  0.00, 0.20, radians( 30), radians( 25), radians(0)],
      [0.32,  0.10, 0.20, radians( 30), radians( 25), radians(0)],

      [0.20,  0.15, 0.20, radians( 30), radians( 25), radians(0)],
      [0.20,  0.00, 0.20, radians( 30), radians( 25), radians(0)],
      [0.20, -0.15, 0.20, radians(  0), radians( 25), radians(0)],

      [0.15, -0.10, 0.10, radians(  0), radians( 25), radians(0)],
      [0.15,  0.05, 0.10, radians( 30), radians( 25), radians(0)],
      [0.15,  0.20, 0.10, radians( 30), radians( 25), radians(0)],
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
  },
  'a_bot_camera': {
    'a_bot': [
      [ 0.00, -0.20, 0.20, radians(90), radians( 70), radians( 90)],
      [ 0.00, -0.17, 0.20, radians(90), radians( 80), radians( 90)],
      [ 0.00, -0.15, 0.20, radians(90), radians( 90), radians( 90)],
      [ 0.00, -0.13, 0.20, radians(90), radians(110), radians( 90)],
      [ 0.00, -0.10, 0.20, radians(90), radians(110), radians( 90)],
      [-0.05, -0.15, 0.20, radians(90), radians( 90), radians( 70)],
      [-0.02, -0.15, 0.20, radians(90), radians( 90), radians( 80)],
      [ 0.02, -0.15, 0.20, radians(90), radians( 90), radians(100)],
      [ 0.05, -0.15, 0.20, radians(90), radians( 90), radians(110)],
    ]
  }
}


def get_service_proxy(service_name, camera_name, robot_name):
  """Return ROS service proxy"""
  cs = "/{}/".format(camera_name)
  if camera_name == "a_bot_camera":
    ns = "/o2as_easy_handeye_{}_eye_on_hand/".format(robot_name)
  else:
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
    self.nimages       = 0
    self.camera_name   = camera_name
    
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
    self.robot_name = robot_name
    group = self.groups[robot_name]

    # Set `_ee_link` as end effector wrt `_base_link` of the robot
    #group.set_pose_reference_frame(robot_name + "_base_link")
    group.set_pose_reference_frame("workspace_center")
    group.set_end_effector_link(robot_name + "_ee_link")
    group.set_end_effector_link(robot_name + "_gripper_tip_link")
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


    # ### How to define poses/positions for calibration
    # # 1. From rostopic echo /joint_states (careful with the order of the joints)
    # joint_pose = [-0.127, 0.556, 0.432, -1.591,  0.147, -0.285]

    # # 2. From Rviz, ee frame position after planning (Open TF Frames, unfold the frame a_bot_ee_link)
    # poseStamped.pose.position.x = -0.16815
    # poseStamped.pose.position.y = -0.10744
    # poseStamped.pose.position.z = 1.1898
    # poseStamped.pose.orientation.x = -0.531
    # poseStamped.pose.orientation.y = 0.5318
    # poseStamped.pose.orientation.z = 0.46652
    # poseStamped.pose.orientation.w = 0.46647

    # # 3. Rotate an orientation using TF quaternions
    # quaternion_0 = tf_conversions.transformations.quaternion_from_euler(
    #       pose[3], pose[4], pose[5])
    # q_rotate_30_in_y = tf_conversions.transformations.quaternion_from_euler(0, pi/6, 0)
    # q_rotated = tf_conversions.transformations.quaternion_multiply(quaternion_0, q_rotate_30_in_y)
    # poseStamped.pose.orientation = geometry_msgs.msg.Quaternion(*q_rotated)

  def move(self, pose, speed):
    """Move the end effector"""
    # TODO: check type of `pose`
    print("move to {}".format(pose))
    poseStamped                 = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id = "workspace_center"
    poseStamped.pose.position.x = pose[0]
    poseStamped.pose.position.y = pose[1]
    poseStamped.pose.position.z = pose[2]
    if len(pose) == 6:
      poseStamped.pose.orientation \
        = geometry_msgs.msg.Quaternion(
          *tf_conversions.transformations.quaternion_from_euler(
            pose[3], pose[4], pose[5]))
    else:
      poseStamped.psoe.orientation.x = pose[3]
      poseStamped.psoe.orientation.y = pose[4]
      poseStamped.psoe.orientation.z = pose[5]
      poseStamped.psoe.orientation.w = pose[6]
    [all_close, move_success] = self.go_to_pose_goal(self.robot_name,
                                                     poseStamped, speed,
                                                     move_lin=False)
    if move_success:
      if self.needs_trigger:
        self.start_acquisition()
        rospy.sleep(1)

      if self.needs_calib:
        try:
          self.take_sample()
          sample_list = self.get_sample_list()
          n = len(sample_list.samples.hand_world_samples.transforms)
          print("  took {} (hand-world, camera-marker) samples").format(n)

          # try:
          #   image_msg = rospy.wait_for_message("/a_bot_camera/rgb/image_raw",
          #                                      sensor_msgs.msg.Image, 1.0)
          #   bridge = CvBridge()
          #   cv2_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
          #   cv2.imwrite("camera_image-{}.jpeg".format(self.nimages), cv2_img)
          #   self.nimages += 1
          # except CvBridgeError, e:
          #   print(e)
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
      if self.camera_name == "a_bot_camera":
        self.move(keypose, speed)
      else:
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
    
    assert(camera_name in {"a_phoxi_m_camera", "a_bot_camera"})
    assert(robot_name  in {"a_bot", "b_bot", "c_bot"})

    routines = HandEyeCalibrationRoutines(camera_name, robot_name,
                                          needs_trigger, needs_calib)

    print("=== Calibration started for {} + {} ===".format(camera_name,
                                                           robot_name))
    speed      = 1
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
