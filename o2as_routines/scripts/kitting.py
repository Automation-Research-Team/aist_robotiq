#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
import tf
from math import pi

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg

from o2as_routines.base import O2ASBaseRoutines


class KittingClass(O2ASBaseRoutines):
  """
  This contains the routine used to run the kitting task. See base.py for shared convenience functions.
  """
  def __init__(self):
    super(KittingClass, self).__init__()
    self.set_up_item_parameters()
    rospy.sleep(.5)

  def set_up_item_parameters(self):
    self.item_names = []
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
    # 
    

  ################ ----- Routines  
  ################ 
  ################ 

  def pick(self, robot_name, object_pose, grasp_height, speed_fast, speed_slow, approach_height=0.03):

    self.publish_marker(object_pose, "aist_vision_result")
    if robot_name=="b_bot":
      self.groups[robot_name].set_end_effector_link(robot_name + '_dual_suction_gripper_pad_link')

    rospy.loginfo("Going above object to pick")
    approach_pose = geometry_msgs.msg.PoseStamped()
    approach_pose = copy.deepcopy(object_pose)
    approach_pose.pose.position.z += 0.03
    approach_pose.pose.orientation.x = -0.5
    approach_pose.pose.orientation.y = 0.5
    approach_pose.pose.orientation.z = 0.5
    approach_pose.pose.orientation.w = 0.5
    res = self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)

    rospy.loginfo("Moving down to object")
    self.go_to_pose_goal(robot_name, object_pose, speed=speed_slow, high_precision=True)

    rospy.loginfo("suction on")
    rospy.sleep(2)

    rospy.loginfo("Going back up")
    self.go_to_pose_goal(robot_name, approach_pose, speed=speed_fast)

  def kitting_task(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")


    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "set1_bin2_1"
    ps.pose.position.z = 0.05
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pi/4, pi/4, 0))
    
    # ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    self.pick("b_bot", ps, ps.pose.position.z, 1.0, 0.05, ps.pose.position.z+0.03)
    # self.publish_marker(ps, "aist_vision_result")

    # TODO

if __name__ == '__main__':
  try:
    kit = KittingClass()
    kit.set_up_item_parameters()
    
    kit.kitting_task()

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
