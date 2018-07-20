#!/usr/bin/env python
import rospy
from phoxi_camera.srv import *
from std_srvs.srv import *

if __name__ == '__main__':
    rospy.init_node('pick_and_place', anonymous=True)
    while not rospy.is_shutdown():
        