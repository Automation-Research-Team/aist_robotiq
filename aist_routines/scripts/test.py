#!/usr/bin/env python

import rospy, collections
from aist_routines import AISTBaseRoutines
from aist_routines import msg as amsg

if __name__ == '__main__':

    rospy.init_node("test", anonymous=True)

    robots   = rospy.get_param("~robots")
    cameras  = rospy.get_param("~cameras")
    grippers = {}

    for robot_name, robot in robots.items():
        if "grippers" in robot:
            grippers.update(robot["grippers"])
        if "cameras" in robot:
            cameras.update(robot["cameras"])

    # for camera_name, camera in cameras.items():
    #     print camera_name, ':', camera

    # for gripper_name, gripper in grippers.items():
    #     print gripper_name, ':', gripper

    gripper_list = {}
    camera_list = {}
    for robot_name, robot in robots.items():
        if "grippers" in robot:
            for gripper in robot["grippers"].values():
                gripper_list[robot_name] = gripper
        if "cameras" in robot:
            for camera in robot["cameras"].values():
                camera_list[robot_name] = camera

    print gripper_list
    print camera_list
