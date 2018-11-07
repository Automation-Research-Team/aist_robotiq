#!/usr/bin/env python

from math import pi
import copy

import rospy
import tf
import tf_conversions
import geometry_msgs.msg

from aist_routines.base import AISTBaseRoutines

class KittingClass(AISTBaseRoutines):
    def __init__(self):
        super(KittingClass, self).__init__()
        self.use_real_robot = rospy.get_param("use_real_robot", False)

        rospy.loginfo("Kitting class is staring up!")

if __name__ == '__main__':

    rospy.init_node("Kitting")

    try:
        kit = KittingClass()

        while not rospy.is_shutdown():
            rospy.loginfo("x: Exit")

            i = raw_input()
            if i == '1':
                pass
            elif i == 'x':
                break
        print("================ Done!")
    except rospy.ROSInterruptException:
        pass
