import sys, os, collections
import rospy, rospkg, rosparam
import actionlib
import numpy as np
import std_msgs
import base
from aist_localization import msg as amsg

#########################################################################
#  class LocalizationClient                                             #
#########################################################################
class LocalizationClient(object):

    def __init__(self):
        super(LocalizationClient, self).__init__()

        self._localize = actionlib.SimpleActionClient(
                           "/aist_localization/localize", amsg.LocalizeAction)

    def send_goal(self):
        goal = amsg.LocalizeGoal()
        goal.object_name = object_name
        goal.timeout     = timeout
        goal.number_of_poses = number_of_poses
        self._localize.send_goal(goal, feedback_cb=self._feedback_cb)

    def wait_for_result(self):
        self._localize.wait_for_result()
        result = self._localize.get_result()
        return (self._compute_poses(result), result.overlabs)

    def cancel_goal(self):
        self._localize.cancel_goal()

    def _feedback_cb(self, feedback):
        pass

    def _compute_poses(self, res):
