#!/usr/bin/env python

import sys, socket, rospy
from aist_robotiq.cmodel_urcap import RobotiqCModelURCap
from aist_robotiq              import msg as amsg

def mainLoop(ur_address):
    name = rospy.get_name()

    # Gripper is a C-Model that is connected to a UR controller
    # with the Robotiq URCap installed.
    # Commands are published to port 63352 as ASCII strings.
    rospy.loginfo("(%s) connecting to gripper[%s:63352]" % (name, ur_address))
    gripper = RobotiqCModelURCap(ur_address)

    # The Gripper status
    pub = rospy.Publisher('/status', amsg.CModelStatus, queue_size=3)
    # The Gripper command
    rospy.Subscriber('/command', amsg.CModelCommand, gripper.sendCommand)

    if not gripper.is_active():
        rospy.loginfo("(%s) activating gripper" % name)
        gripper.activate(auto_calibrate=False)

    rospy.loginfo("(%s) gripper ready" % name)

    while not rospy.is_shutdown():
        # Get and publish the Gripper status
        status = gripper.getStatus()
        pub.publish(status)
        # Wait a little
        rospy.sleep(0.1)

if __name__ == '__main__':
    rospy.init_node('cmodel_urcap_driver')
    try:
        mainLoop(sys.argv[1])
    except rospy.ROSInterruptException: pass