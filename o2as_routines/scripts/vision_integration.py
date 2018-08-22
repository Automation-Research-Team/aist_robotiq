#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf_conversions
from math import pi

from o2as_msgs.srv import *
import actionlib
import o2as_msgs.msg

from o2as_routines.base import O2ASBaseRoutines
from o2as_routines.vision import VisionProxy

import tf2_ros
import tf2_geometry_msgs

class VisionIntegration(O2ASBaseRoutines):
  """
  This contains the routine used to run the vi task.
  """
  def __init__(self):
    super(VisionIntegration, self).__init__()
    # self.action_client.wait_for_server()
    rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

  ################ ----- Routines  
  ################ 
  ################ 
  def pick(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command = "", approach_height = 0.03):
    rospy.loginfo("pick() begin")
    rospy.loginfo("robotname = " + robotname)
    rospy.loginfo("object_pose: ")
    rospy.loginfo(object_pose)
    rospy.loginfo("grasp_height = " + str(grasp_height))
    rospy.loginfo("speed_fast = " + str(speed_fast))
    rospy.loginfo("speed_slow = " + str(speed_slow))

    self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

  def place(self,robotname, object_pose, place_height, speed_fast, speed_slow, gripper_command = "", approach_height = 0.05, lift_up_after_place = True):
    rospy.loginfo("Going above place target")

if __name__ == '__main__':
  try:
    vision = VisionProxy()
    vi = VisionIntegration()
    
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    vi.groups["a_bot"].set_goal_tolerance(.0001) 
    vi.groups["a_bot"].set_planning_time(3) 
    vi.groups["a_bot"].set_num_planning_attempts(10)
    vi.go_to_named_pose("home_c", "c_bot")
    vi.go_to_named_pose("home_b", "b_bot")
    vi.go_to_named_pose("home_a", "a_bot")

    i = raw_input("Enter the number of the part to be performed: ")
    i = int(i)
    while(i):
      if i > 0:
        rospy.logdebug("find object")
        expected_position = geometry_msgs.msg.PoseStamped()
        object_id = str(i)
        item_pose = vision.find_object(expected_position, position_tolerance = 0.2, object_id = object_id, camera = "b_bot_camera")
        if item_pose == None:
          continue

        transform = tf_buffer.lookup_transform("mat", #target frame
                                              item_pose.header.frame_id, #source frame
                                              rospy.Time(0), #get the tf at first available time
                                              rospy.Duration(1.0)) #wait for 1 second
        pick_pose = tf2_geometry_msgs.do_transform_pose(item_pose, transform)

        downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))
        pick_pose.pose.orientation = downward_orientation
        place_pose = pick_pose
        
        rospy.logdebug("pick object")
        vi.pick("b_bot", pick_pose, grasp_height = 0.03, approach_height = 0.05,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner")

        rospy.logdebug("place object")
        vi.place("b_bot", place_pose, place_height = 0.03, approach_height = 0.05,
                                speed_fast = 0.2, speed_slow = 0.02, gripper_command="easy_pick_only_inner",
                                lift_up_after_place = False)

        # rospy.logdebug("goto home position")
        # vi.go_to_named_pose("home_c", "c_bot")
        # vi.go_to_named_pose("home_b", "b_bot")
        # vi.go_to_named_pose("home_a", "a_bot")
      
      i = raw_input("the number of the part")
      i = int(i)
    print "============ Done!"
    
  except rospy.ROSInterruptException:
    pass
