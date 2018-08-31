#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CameraInfo

# Initialize ROS node
rospy.init_node("convert_phoxi_camera_info_gazebo")

# Publisher for conversion of camera info messages
pub = rospy.Publisher("/a_phoxi_m_camera/camera_info2", CameraInfo,
                      queue_size=10)

# Callback
def callback(message):
  P = list(message.P)
  P[3] = float(0.0)
  message.P = tuple(P)
  pub.publish(message)

# Start converter
_ = rospy.Subscriber("/a_phoxi_m_camera/camera_info", CameraInfo, callback)
rospy.spin()
