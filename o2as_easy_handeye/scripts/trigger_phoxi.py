#!/usr/bin/env python
# This script run a ROS node which triggers Phoxi at 1Hz frequency.

import rospy
from std_srvs.srv import Trigger
from o2as_phoxi_camera.srv import SetInt

# Initialize ROS node
rospy.init_node("trigger_phoxi")

# ROS services proxies
start_acquisition = rospy.ServiceProxy("/o2as_phoxi_camera/start_acquisition",
                                       Trigger)
trigger_frame = rospy.ServiceProxy("/o2as_phoxi_camera/trigger_frame", Trigger)
get_frame = rospy.ServiceProxy("/o2as_phoxi_camera/get_frame", SetInt)

# Initialize Phoxi services
start_acquisition()
rospy.sleep(1.0)
trigger_frame()

# Trigger loop
while True:
    trigger_frame()
    get_frame(0)
    rospy.sleep(0.5)
