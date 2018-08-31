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

    

  def kitting_task(self):
    self.go_to_named_pose("home", "c_bot")
    self.go_to_named_pose("home", "b_bot")
    self.go_to_named_pose("home", "a_bot")


    ps = geometry_msgs.msg.PoseStamped()
    ps.header.frame_id = "workspace_center"
    ps.pose.position.z = 0.25
    ps.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
    self.publish_marker(ps, "aist_vision_result")

    # TODO
    


if __name__ == '__main__':
  try:
    kit = KittingClass()
    kit.set_up_item_parameters()
    
    kit.kitting_task()

    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
