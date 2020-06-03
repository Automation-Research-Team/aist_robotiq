#!/usr/bin/env python

import rospy, collections
import aist_routines.base as base
from aist_routines.base import AISTBaseRoutines
from aist_routines      import msg as amsg

if __name__ == '__main__':

    rospy.init_node("test", anonymous=True)

    robots = rospy.get_param("~robots")

    for robot in robots:
        for gripper in robot["grippers"]:
            print("Gripper: %s", gripper)
        for camera in robot["cameras"]:
            print("Camera: %s", camera)
