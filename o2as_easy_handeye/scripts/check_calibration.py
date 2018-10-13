#!/usr/bin/env python

import sys
import os
import copy
import rospy

import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import tf_conversions

from math import radians, degrees

#from std_msgs.msg import String
#from std_srvs.srv import Empty
from std_srvs.srv import Trigger
#from o2as_phoxi_camera.srv import GetFrame

from o2as_routines.base import O2ASBaseRoutines
from o2as_aruco_ros.msg import Corners



######################################################################
#  class VisitRoutines                                               #
######################################################################
class VisitRoutines(O2ASBaseRoutines):
  """Wrapper of MoveGroupCommander specific for this script"""
  def __init__(self, camera_name, robot_name):
    super(VisitRoutines, self).__init__()
    
    cs = "/{}/".format(camera_name)
    self.start_acquisition = rospy.ServiceProxy(cs + "start_acquisition",
                                                Trigger)
    self.stop_acquisition  = rospy.ServiceProxy(cs + "stop_acquisition",
                                                Trigger)

    ## Initialize `moveit_commander`
    self.robot_name = robot_name
    group = self.groups[robot_name]

    # Set `_ee_link` as end effector wrt `_base_link` of the robot
    #group.set_pose_reference_frame(robot_name + "_base_link")
    group.set_pose_reference_frame("workspace_center")
    group.set_end_effector_link(robot_name + "_gripper_tip_link")

    # Trajectory publisher
    display_trajectory_publisher = rospy.Publisher(
      '/move_group/display_planned_path',
      moveit_msgs.msg.DisplayTrajectory,
      queue_size=20
    )

    # Logging
    print("============ Reference frame: %s" % group.get_planning_frame())
    print("============ End effector: %s"    % group.get_end_effector_link())

    
  def move(self, speed):
    self.start_acquisition()
    position = rospy.wait_for_message("/aruco_tracker/position",
                                      geometry_msgs.msg.Vector3Stamped, 10)
    self.stop_acquisition()

    print("move to {}".format(position.vector))
    poseStamped = geometry_msgs.msg.PoseStamped()
    poseStamped.header.frame_id = "workspace_center"
    poseStamped.pose.position.x = position.vector.x
    poseStamped.pose.position.y = position.vector.y
    poseStamped.pose.position.z = position.vector.z - 0.0285 + 0.05
    poseStamped.pose.orientation \
      = geometry_msgs.msg.Quaternion(
        *tf_conversions.transformations.quaternion_from_euler(
          radians(180), radians(90), radians(90)))
    [all_close, move_success] = self.go_to_pose_goal(self.robot_name,
                                                     poseStamped, speed,
                                                     move_lin=False)
    rospy.sleep(1)
    poseStamped.pose.position.z = position.vector.z -0.0285 + 0.01
    [all_close, move_success] = self.go_to_pose_goal(self.robot_name,
                                                     poseStamped, speed,
                                                     move_lin=False)

    # corners = rospy.wait_for_message("/aruco_tracker/corners",
    #                                  Corners, 1)
    # for corner in enumerate(corneres):
    #   print("move to {}".format(corner.point))
    #   poseStamped = geometry_msgs.msg.PoseStamped()
    #   poseStamped.pose.position.x = corner.point.x
    #   poseStamped.pose.position.y = corner.point.y
    #   poseStamped.pose.position.z = corner.point.z + 0.1
    #   poseStamped.pose.orientation \
    #     = geometry_msgs.msg.Quaternion(
    #         *tf_conversions.transformations.quaternion_from_euler(
    #             radians(90), radians(90), radians(90)))
    #   [all_close, move_success] = self.go_to_pose_goal(self.robot_name,
    #                                                    poseStamped, speed,
    #                                                    move_lin=False)
    #   rospy.sleep(1)
    #   poseStamped.pose.position.z = corner.point.z
    #   [all_close, move_success] = self.go_to_pose_goal(self.robot_name,
    #                                                    poseStamped, speed,
    #                                                    move_lin=False)
        

  def go_home(self):
    self.go_to_named_pose("home", self.robot_name)


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
  camera_name = sys.argv[1]
  robot_name  = sys.argv[2]
    
  assert(camera_name in {"a_phoxi_m_camera", "a_bot_camera"})
  assert(robot_name  in {"a_bot", "b_bot", "c_bot"})

  routines = VisitRoutines(camera_name, robot_name)
  speed = 0.05
  routines.run(speed)

