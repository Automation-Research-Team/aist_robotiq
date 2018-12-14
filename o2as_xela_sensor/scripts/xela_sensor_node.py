#!/usr/bin/env python

import os
import copy
import rospy
import rospkg
rospack = rospkg.RosPack()

import actionlib
from o2as_xela_sensor.msg import *
from o2as_xela_sensor.srv import *
from o2as_xela_sensor.xela_sensor import *

class XelaSensorNode(object):
  def __init__(self):
    # Connect to the sensor and trigger to start acquisition
    board_id = rospy.get_param("board_id", 1)
    self._sensor = XelaSensor(board_id)
    self._sensor.start_data_acquisition()
    
    # Calibrate once
    data_dir = os.path.join(rospack.get_path("o2as_xela_sensor"), "data")
    filename = os.path.join(data_dir, "log{}.csv".format(board_id))
    self._sensor.calibrate(sample_num=100, filename=filename)

    # Publishers
    self._pub_base = rospy.Publisher("~base", XelaSensorStamped, queue_size=1)
    self._pub_data = rospy.Publisher("~data", XelaSensorStamped, queue_size=1)
    self._board_id = board_id

    # Calibrate action (for re-calibration)
    self._calibrate_action_name = "~calibrate"
    self._calibrate_action_server = actionlib.SimpleActionServer(self._calibrate_action_name, CalibrateAction, execute_cb=self.calibrate_action_callback, auto_start = False)
    self._calibrate_action_server.start()
    rospy.loginfo('Action server {} started.'.format(self._calibrate_action_name))
    self._calibrate_action_result = CalibrateResult()

  def calibrate_action_callback(self, goal):
    rospy.loginfo('Executing {}. request sent:'.format(self._calibrate_action_name))
    rospy.loginfo(goal)
    res = self._sensor.calibrate(sample_num=goal.sample_num, filename=goal.log_filename)
    self._calibrate_action_result.success = res
    self._calibrate_action_result.base = self._sensor.base
    self._calibrate_action_server.set_succeeded(self._calibrate_action_result)
    rospy.loginfo('Action server {} finished.'.format(self._calibrate_action_name))

  def free_run(self):
    while not rospy.is_shutdown():
      # Publish base (is this necessary?)
      base_msg = XelaSensorStamped()
      base_msg.board_id = self._board_id
      base_msg.header.stamp = rospy.Time.now()
      base_msg.data = self._sensor.base
      self._pub_base.publish(base_msg)

      # Publish data
      data_msg = XelaSensorStamped()
      data_msg.board_id = self._board_id
      data_msg.header.stamp = rospy.Time.now()
      data_msg.data = self._sensor.get_data()
      self._pub_data.publish(data_msg)
      rospy.sleep(0.001)

if __name__ == '__main__':
  rospy.init_node('xela_robotics_sensor', anonymous=True, log_level=rospy.DEBUG)
  node = XelaSensorNode()
  node.free_run()
