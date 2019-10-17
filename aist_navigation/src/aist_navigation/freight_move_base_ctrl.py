#!/usr/bin/env python
import rospy
from check_reproducibility import MoveBaseClient

"""
Move freight to backyard without map

Prerequisities (HW):
- Ubuntu on freight is started (or rebooted) near the fetch.

How to start:
1. Launch below file.
`$ roslaunch aist_navigation move_base_mapless.launch`

2. Run this script.
`$ rosrun freight_move_base_ctrl.py`
"""

if __name__ == "__main__":
    rospy.init_node("move_base_test")
    client = MoveBaseClient()
    waypoints = [
        [1.8, 0, 0, 'odom']
    ]

    for waypoint in waypoints:
        result = client.move_to(*waypoint)
        rospy.loginfo(result)
