#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from o2as_fastener_gripper.srv import *

class FastenerGripperControllerClient:
    def __init__(self, server_name):
        fasten_service_name = server_name+'/Fasten'
        rospy.wait_for_service(fasten_service_name)
        self.fasten = rospy.ServiceProxy(fasten_service_name, Fasten)
        stop_service_name = server_name+'/Stop'
        self.stop = rospy.ServiceProxy(stop_service_name, Stop)

    def fasten(self):
        try:
            res = self.fasten()
            return res.result
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)

    def stop(self):
        try:
            res = self.stop()
            return res.result
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)


if __name__ == '__main__':
    rospy.init_node('fastener_gripper_controller_test')

    fastener_grippers = [
        FastenerGripperControllerClient("fg1"),
        # FastenerGripperControllerClient("fg2"),
        # FastenerGripperControllerClient("fg3"),
    ]

    for c in fastener_grippers:
        c.fasten()
        # rospy.sleep(1.0)
        # c.stop()

    rospy.spin()
