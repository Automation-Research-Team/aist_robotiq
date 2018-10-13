#!/usr/bin/env python

import os
import cv2.aruco
import numpy as np

import rospy
import rospkg
rospack = rospkg.RosPack()

import tf
import tf2_ros
import geometry_msgs.msg
from o2as_aruco_marker_detection.camera_adapter import *
from o2as_aruco_marker_detection.marker_detection import MarkerDetection

class MarkerDetectionNode(MarkerDetection):
  def __init__(self):
    super(MarkerDetectionNode, self).__init__()

    # connect to the camera
    camera_name = rospy.get_param("~camera_name")
    camera_type = rospy.get_param("~camera_type")
    marker_id   = rospy.get_param("~marker_id")
    self.camera = CameraClient(camera_name, camera_type)

    # get image and detect markers
    while not rospy.is_shutdown():
      cloud, texture = self.camera.get_frame(publish=False)
      res = self.detect_marker(cloud, texture, marker_id)
      # rospy.logdebug(str(res))

    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
  rospy.init_node('marker_detection', anonymous=True, log_level=rospy.DEBUG)
  node = MarkerDetectionNode()
  rospy.spin()
