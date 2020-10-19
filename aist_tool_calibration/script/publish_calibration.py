#!/usr/bin/env python2

import os
import rospy
from tf import TransformBroadcaster, TransformListener, transformations as tfs
from geometry_msgs import msg as gmsg
from math import degrees

#########################################################################
#  class ToolCalibrationPublisher                                       #
#########################################################################
class ToolCalibrationPublisher(object):
    def __init__(self):
        super(ToolCalibrationPublisher, self).__init__()

        self._broadcaster = TransformBroadcaster()

        parent = rospy.get_param("~parent")
        child  = rospy.get_param("~child")
        T      = rospy.get_param("~transform")

        self._transform = gmsg.TransformStamped()
        self._transform.header.frame_id = parent
        self._transform.child_frame_id  = child
        self._transform.transform \
            = gmsg.Transform(gmsg.Vector3(T["x"], T["y"], T["z"]),
                             gmsg.Quaternion(
                                 T["qx"], T["qy"], T["qz"], T["qw"]))

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self._transform.header.stamp = rospy.Time.now()
            self._broadcaster.sendTransformMessage(self._transform)
            rate.sleep()

#########################################################################
#  main part                                                            #
#########################################################################
if __name__ == "__main__":
    rospy.init_node("publish_tool_calibration")

    cp = ToolCalibrationPublisher()
    cp.run()
