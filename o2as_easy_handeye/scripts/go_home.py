#!/usr/bin/env python

import rospy
import argparse
from o2as_routines.base import O2ASBaseRoutines
from aist_routines.base import AISTBaseRoutines


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Move robot to named pose')
    parser.add_argument('-C', '--config',
                        action='store', nargs='?',
                        default='aist', type=str, choices=None,
                        help='configuration name', metavar=None)
    parser.add_argument('-r', '--robot_name',
                        action='store', nargs='?',
                        default='b_bot', type=str, choices=None,
                        help='robot name', metavar=None)
    parser.add_argument('-p', '--pose',
                        action='store', nargs='?',
                        default='home', type=str, choices=None,
                        help='pose name', metavar=None)

    args = parser.parse_args()

    if args.config == "o2as":
        baseRoutines = O2ASBaseRoutines()
    else:
        baseRoutines = AISTBaseRoutines()

    baseRoutines.go_to_named_pose(args.robot_name, args.pose)
