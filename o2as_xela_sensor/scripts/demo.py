#!/usr/bin/env python

import os
import cv2
import copy
import numpy as np

import rospy
import rospkg
rospack = rospkg.RosPack()
import actionlib
from o2as_xela_sensor.msg import *
import o2as_xela_sensor.srv

taxel_rows = 4
taxel_cols = 4
taxel_num = taxel_rows*taxel_cols

class XelaSensorClient(object):
  def __init__(self):
    self._base = [0] * taxel_num * 3
    self._data = [0] * taxel_num * 3
    self._sub_data = rospy.Subscriber("data", XelaSensorStamped, self.data_callback)

  def calibrate(self, sample_num, log_filename):
    calibrate_action_client = actionlib.SimpleActionClient('calibrate', o2as_xela_sensor.msg.CalibrateAction)
    calibrate_action_client.wait_for_server()
    goal = o2as_xela_sensor.msg.CalibrateGoal()
    goal.sample_num = sample_num
    goal.log_filename = log_filename
    calibrate_action_client.send_goal(goal)
    calibrate_action_client.wait_for_result()

  def get_baseline(self):
    get_baseline = rospy.ServiceProxy("get_baseline", o2as_xela_sensor.srv.GetBaseline)
    response = get_baseline()
    return response.data

  def data_callback(self, msg_in):
    self._data = copy.deepcopy(msg_in.data)

  @property
  def data(self):
    return self._data

class XelaSensorDemo(XelaSensorClient):
  def __init__(self):
    super(XelaSensorDemo, self).__init__()

    # Run calibration and get baseline of the sensor (if necessary)
    data_dir = os.path.join(rospack.get_path("o2as_xela_sensor"), "data")
    self.calibrate(sample_num=100, log_filename=os.path.join(data_dir, "calibration_log.csv"))
    base = self.get_baseline()

    # Constants
    width  = 800 # width of the image
    height = 800 # height of the image
    margin = 160 # margin of the taxel in the image
    pitch  = 160 # pitch between taxels in the image
    scale  = 25  # scale from the sensor data to the image
    tz     = 12  # default size of the circle
    color  = (0,255,0,255)

    # Open window to display state of taxels on the board
    img = np.zeros((height,width,3), np.uint8)
    cv2.namedWindow('xela-sensor', cv2.WINDOW_NORMAL)

    while not rospy.is_shutdown():
      # Read new data from the sensor and update z pos
      
      diff = np.array(self.data) - np.array(base)
      dx = diff.reshape((taxel_num,3))[:,0] / scale
      dy = diff.reshape((taxel_num,3))[:,1] / scale
      dz = diff.reshape((taxel_num,3))[:,2] / scale

      # Draw circles that represents probes of tactile sensor
      k = 0
      for j in range(taxel_rows):
        for i in range(taxel_cols):
          x = np.clip(width-margin-i*pitch-dx[k], 0, width)
          y = np.clip(      margin+j*pitch+dy[k], 0, height)
          z = np.clip(                  tz+dz[k], 0, 100)
          cv2.circle(img, (int(x), int(y)), int(z), color, -1)
          k = k+1

      cv2.imshow("xela-sensor", img)
      if cv2.waitKey(1) > 0:
        break
      img[:]=(0,0,0)

  def __del__(self):
    cv2.destroyAllWindows()

if __name__ == '__main__':
  rospy.init_node('xela_sensor_demo', anonymous=True, log_level=rospy.DEBUG)
  node = XelaSensorDemo()
  rospy.spin()
