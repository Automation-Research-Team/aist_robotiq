#!/usr/bin/env python

import rospy
import actionlib
import std_msgs.msg

import robotiq_msgs.msg

import o2as_msgs.msg
import o2as_msgs.srv

######################################################################
#  global functions                                                  #
######################################################################
def clamp(x, x_min, x_max):
    return min(max(x, x_min), x_max)

######################################################################
#  class GripperBase                                                 #
######################################################################
class GripperBase(object):
    def __init__(self, name, tip_link, grasp_offset=0, timeout=0):
        self._name         = name
        self._tip_link     = tip_link
        self._grasp_offset = grasp_offset
        self._timeout      = timeout

    @property
    def name(self):
        return self._name

    @property
    def tip_link(self):
        return self._tip_link

    @property
    def grasp_offset(self):
        return self._grasp_offset

    @grasp_offset.setter
    def grasp_offset(self, offset):
        self._grasp_offset = offset

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, t):
        self._timeout = t

    def pregrasp(self):
        return True

    def grasp(self):
        return True

    def release(self):
        return True

######################################################################
#  class Robotiq85Gripper                                            #
######################################################################
class Robotiq85Gripper(GripperBase):
    def __init__(self, prefix='a_bot_', grasp_offset=-0.005,
                 force=5.0, velocity=0.1, timeout=6.0):
        super(Robotiq85Gripper, self).__init__(str(prefix) +
                                               'robotiq_85_gripper',
                                               str(prefix) +
                                               'robotiq_85_tip_link',
                                               grasp_offset, timeout)
        self._client = actionlib.SimpleActionClient(
                           str(prefix) + 'gripper/gripper_action_controller',
                           robotiq_msgs.msg.CModelCommandAction)
        self._goal   = robotiq_msgs.msg.CModelCommandGoal()
        self._goal.force    = force
        self._goal.velocity = velocity
        self._goal.position = 0.0

    @property
    def force(self):
        return self._goal.force

    @force.setter
    def force(self, f):
        self._goal.force = f

    @property
    def velocity(self):
        return self._goal.velocity

    @velocity.setter
    def velocity(self, v):
        self._goal.velocity = v

    def pregrasp(self):
        return self.open(0.085)

    def grasp(self):
        return self.open(0.0)

    def release(self):
        return self.open(0.085)

    def open(self, position):
        self._goal.position = clamp(position, 0.0, 0.085)
        self._client.send_goal(self._goal)
        # This sleep is necessary for robotiq gripper to work just as intended.
        rospy.sleep(.5)
        self._client.wait_for_result(rospy.Duration(self.timeout))
        result = self._client.get_result()
        return result.reached_goal

######################################################################
#  class SuctionGripper                                              #
######################################################################
class SuctionGripper(GripperBase):
    def __init__(self, prefix='b_bot_single_', grasp_offset=0, eject=False,
                 timeout=2.0):
        super(SuctionGripper, self).__init__(str(prefix) + 'suction_gripper',
                                             str(prefix) +
                                             'suction_gripper_pad_link',
                                             grasp_offset, timeout)
        self._client    = actionlib.SimpleActionClient(
                              'o2as_fastening_tools/suction_control',
                              o2as_msgs.msg.SuctionControlAction)
        self._sub       = rospy.Subscriber("suction_tool/screw_suctioned",
                                           std_msgs.msg.Bool,
                                           self._state_callback)
        self._suctioned = False
        self._eject     = eject  # blow when releasing
        self._goal      = o2as_msgs.msg.SuctionControlGoal()
        self._goal.fastening_tool_name = "suction_tool"
        self._goal.turn_suction_on     = False
        self._goal.eject_screw         = False

    @property
    def eject(self):
        return self._eject

    @eject.setter
    def eject(self, e):
        self._eject = e

    @property
    def suctioned(self):
        return self._suctioned

    def pregrasp(self):
        return self._send_command(True)

    def grasp(self):
        return self._send_command(True) and self._suctioned

    def release(self):
        return self._send_command(False)

    def _send_command(self, turn_suction_on):
        self._goal.turn_suction_on = turn_suction_on
        self._goal.eject_screw     = not turn_suction_on and self._eject
        self._client.send_goal(self._goal)
        self._client.wait_for_result(rospy.Duration(self.timeout))
        result = self._client.get_result()
        return result.success

    def _state_callback(self, msg):
        self._suctioned = msg.data
