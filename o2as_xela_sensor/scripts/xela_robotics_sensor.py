#!/usr/bin/env python

import rospy
import copy
import can
import cv2
import numpy as np
import time

def get_data(bus, data):
  while not rospy.is_shutdown():
    id_list   = [0] * 16
    data_list = [0] * 16

    success = False
    for i in range(0,16):
      recvmsg = bus.recv()
      if recvmsg.arbitration_id >= 1808: 
        id_list[recvmsg.arbitration_id - 1808] = recvmsg.arbitration_id
        if recvmsg.data[1] < 250:
          success = True
          data_list[recvmsg.arbitration_id - 1808] = recvmsg.data
        else:
          success = False
          break
  
    if success:
      for i in range(16):
        if id_list[i] != 0:
          # update data if available
          data[i*3+0] = data_list[i][1] << 8 | data_list[i][2]
          data[i*3+1] = data_list[i][3] << 8 | data_list[i][4]
          data[i*3+2] = data_list[i][5] << 8 | data_list[i][6]
      break

def draw_circles(img, z, color):
  height, width = img.shape[0:2]
  n = 4
  k = 0
  for j in range(n):
    for i in range(n):
      cv2.circle(img, (100+j*160, 100+i*160), int(z[k]), color, -1)
      k = k+1

def demo():
  # connect to the sensor and trigger to start acquisition
  bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
  msg = can.Message(arbitration_id=0x201, data=[7], extended_id=False)
  try:
    bus.send(msg)
  except can.CanError:
    rospy.logerr("Message NOT sent")

  # read 100 frames of data from the sensor and calc base-line
  n_base_samples = 100
  data = np.zeros([48]) # 4x4x3
  base = np.zeros([48]) # 4x4x3
  for k in range(n_base_samples):
    get_data(bus, data)
    base = base + data
  base = base / n_base_samples
  rospy.logdebug("base = " + str(base))

  # display sensor states
  img = np.zeros((680,680,3), np.uint8)
  cv2.namedWindow('Image window', cv2.WINDOW_NORMAL)

  tz = 12
  z = np.array([tz] * 16) # 4x4
  z_scale = 100 # 300

  while not rospy.is_shutdown():
    # read new data from the sensor and update z pos
    get_data(bus, data)
    diff = data - base
    z_diff = diff.reshape((16,3))[:,2] / z_scale
    for i in range(16):
      z[i] = tz + z_diff[i]
    rospy.logdebug("z = " + str(z))

    # draw circles that represents probes of tactile sensor
    draw_circles(img, z, (0,255,0))
    cv2.imshow("Image window", img)
    if cv2.waitKey(1) > 0:
      break
    draw_circles(img, z, (0,0,0)) # img[:]=(0,0,0)

  cv2.destroyAllWindows()

if __name__ == '__main__':
  rospy.init_node('xela_sensor_demo', anonymous=True, log_level=rospy.DEBUG)
  demo()
