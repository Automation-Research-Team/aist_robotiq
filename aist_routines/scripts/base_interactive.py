#!/usr/bin/env python

import rospy
import argparse

from aist_routines.MoveBaseRoutines import MoveBaseRoutines

######################################################################
#  global functions                                                  #
######################################################################
def is_num(s):
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True

######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node("base_interactive", anonymous=True)

    routines = MoveBaseRoutines()

    while not rospy.is_shutdown():
        prompt = "{}>> ".format(
            routines.format_odom(routines.current_odom))
        key = raw_input(prompt)

        if key == 'q':
            break
        elif key == 'move_base':
            x     = float(raw_input("  x     = "))
            y     = float(raw_input("  y     = "))
            theta = float(raw_input("  theta = "))
            routines.move_base(x, y, theta)
        elif key == 'move_base_to_frame':
            routines.move_base_to_frame(raw_input(" frame = "))
