#!/usr/bin/env python

import rospy
import argparse
from aist_routines.CameraClient import RealSenseCamera

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test wrist cameras')
    parser.add_argument('-c',
                        '--camera_name',
                        action='store',
                        nargs='?',
                        default='a_bot_wrist_camera',
                        type=str,
                        choices=None,
                        help='camera name',
                        metavar=None)
    args = parser.parse_args()

    rospy.init_node("test_wrist_cameras")

    camera0 = RealSenseCamera(args.camera_name + "0")
    camera1 = RealSenseCamera(args.camera_name + "1")

    while not rospy.is_shutdown():
        print("Current laser power: #0 = {}, #1 = {}"
              .format(camera0.laser_power, camera1.laser_power))

        n = int(raw_input("camera # >> "))

        if n == 0:
            camera1.continuous_shot(False)
            # camera1.laser_power = 0
            # camera0.laser_power = 1
            camera0.continuous_shot(True)
        else:
            camera0.continuous_shot(False)
            # camera0.laser_power = 0
            # camera1.laser_power = 1
            camera1.continuous_shot(True)
