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
from o2as_vision.client import VisionClient

# THIS FILE IS A STUB, NOT WORKING
class AssemblyRoutines(O2ASBaseRoutines):
  """
  This contains the routine used to run the assembly task.
  """
  def __init__(self):
    super(AssemblyRoutines, self).__init__()
    
    self._parts_dict = dict()
    parts_list = rospy.get_param("parts_list")
    for item in parts_list:
        self._parts_dict[item['name']] = item
        rospy.logdebug(item)

class AssemblyTask():
  def __init__(self):
    self.vision = VisionClient()
    self.routines = AssemblyRoutines()

  ################
  ### subtasks ###
  ################
  
  def subtask_a(self):
    # Assembling the motor to the motor fixing plate with the screws
    # [parts]
    # 3: motor fixing plate
    # 4: motor
    rospy.logerr("subtask_a() not implemented")

  def subtask_b(self):
    # Assembling the motor-shaft-pulley to the motor shaft
    # [parts]
    # 4: motor
    # 5: motor shaft-shaft-pulley
    rospy.logerr("subtask_b() not implemented")

  def subtask_c(self):
    # Assembling the output shaft, screws to secure the output shaft, 
    # washers, double bearings, and the screws to attach the bearings 
    # to the output shaft fixing plate
    # [parts]
    #  2: output shaft fixing plate
    #  7: double bearings
    #  8: output shaft
    # 10: washer
    rospy.logerr("subtask_c() not implemented")

  def subtask_d(self):
    # Assembling the output-shaft-pulley to the output shaft
    # [parts]
    # 11: output-shaft-pulley
    #  8: output shaft
    rospy.logerr("subtask_d() not implemented")

    # pick output-shaft-pulley
    belt_room_center = PoseStamped()
    item_pose = self._proxy.find_object(expected_position=belt_room_center, position_tolerance=0.2, object_id="11", camera="phoxi")
    self._proxy.pick(robot_name="", item_pose=item_pose)

    # insert output-shaft-pulley to output shaft
    target_pose = PoseStamped() # TODO: set pose
    self._proxy.insert(robot_name="")

  def subtask_e(self):
    # Assembling the tension pulley to the output shaft fixing plate 
    # and adjusting the belt tension
    # [parts]
    #  6: belt
    # 13: tension pulley
    rospy.logerr("subtask_e() not implemented")

  def subtask_f(self):
    # Assembling the motor fixing plate and the base plate with the screws to connect both
    # [parts]
    # 1: base plate
    # 3: motor fixing plate
    # *: screw
    rospy.logerr("subtask_f() not implemented")

  def subtask_g(self):
    # Assembling the output shaft fixing plate and the base plate with the screws to connect both
    # [parts]
    # 1: base plate
    # 2: output shaft fixing plate
    # *: screw
    rospy.logerr("subtask_g() not implemented")

  def subtask_h(self):
    # Assembling the belt
    # [parts]
    #  6: belt
    #  5: motor pulley
    # 13: tension pulley
    # 11: output pulley
    rospy.logerr("subtask_h() not implemented")
    
    # pick belt
    rospy.logerr("pick belt not implemented")

    # hang belt to pulleys
    rospy.logerr("hang belt to pulleys not implemented")

  ################
  ### assemble ###
  ################

  def assemble_normal_parts(self):
    rospy.loginfo("assemble belt drive unit with normal parts")

    # motor
    self.subtask_f() # Assembling the motor fixing plate and the base plate with the screws to connect both
    self.subtask_a() # Assembling the motor to the motor fixing plate with the screws
    self.subtask_b() # Assembling the motor-shaft-pulley to the motor shaft
    # output shaft
    self.subtask_g() # Assembling the output shaft fixing plate and the base plate with the screws to connect both
    self.subtask_c() # Assembling the output shaft, screws to secure the output shaft, washers, double bearings, and the screws to attach the bearings to the output shaft fixing plate
    self.subtask_d() # Assembling the output-shaft-pulley to the output shaft
    # belt
    self.subtask_h() # Assembling the belt
    self.subtask_e() # Assembling the tension pulley to the output shaft fixing plate and adjusting the belt tension

  def assemble_surprize_parts(self):
    rospy.loginfo("assemble belt drive unit with surprize parts")
    rospy.logerr("assemble_product_with_surprize_parts() not implemented")

  def assemble(self, product_type):
    rospy.loginfo("assemble belt drive unit")

    if product_type == "normal":
      self.assemble_product_with_normal_parts()
    elif product_type == "surprize":
      self.assemble_product_with_surprize_parts()


if __name__ == '__main__':
  assembly_task = AssemblyTask() 
  try:
    i = raw_input("Enter the number of the part to be performed: ")
    i = int(i)
    while (i):
      if i == 1:
        assembly_task.assemble("normal")
      elif i == 2:
        assembly_task.assemble("surprize")
      i = raw_input("the number of the part")
      i = int(i)
    print "============ Done!"
  except rospy.ROSInterruptException:
    pass
