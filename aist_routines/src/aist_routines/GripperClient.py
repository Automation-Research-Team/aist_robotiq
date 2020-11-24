import rospy
import actionlib
import std_msgs.msg
import std_srvs

from numpy import clip

######################################################################
#  class GripperClient                                               #
######################################################################
class GripperClient(object):
    def __init__(self, name, type, base_link, tip_link, timeout):
        self._name       = name
        self._type       = type
        self._base_link  = base_link
        self._tip_link   = tip_link
        self._timeout    = timeout
        self._parameters = {}

    @staticmethod
    def create(type_name, kwargs):
        ClientClass = globals()[type_name]
        if rospy.get_param('use_real_robot', False):
            return ClientClass(**kwargs)
        else:
            return ClientClass.base(**kwargs)

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    @property
    def base_link(self):
        return self._base_link

    @property
    def tip_link(self):
        return self._tip_link

    @property
    def timeout(self):
        return self._timeout

    @property
    def parameters(self):
        return self._parameters

    @parameters.setter
    def parameters(self, parameters):
        for key, value in parameters.items():
            self._parameters[key] = value

    def pregrasp(self):
        return True

    def grasp(self):
        return True

    def postgrasp(self):
        return True

    def release(self):
        return True

######################################################################
#  class VoidGripper                                                 #
######################################################################
class VoidGripper(GripperClient):
    def __init__(self, name, base_link, timeout=10.0):
        super(VoidGripper, self).__init__(name, 'void',
                                          base_link, base_link, timeout)
        rospy.loginfo('{} initialized.'.format(self.name))

    @staticmethod
    def base(name, base_link, timeout):
        return VoidGripper(name, base_link, timeout)


######################################################################
#  class GenericGripper                                              #
######################################################################
class GenericGripper(GripperClient):
    def __init__(self, name, action_ns, base_link, tip_link, timeout=10.0,
                 min_gap=0.0, max_gap=0.1, max_effort=5.0):
        from control_msgs.msg import GripperCommandAction, GripperCommandGoal

        super(GenericGripper, self).__init__(name, 'two_finger',
                                             base_link, tip_link, timeout)
        self._client = actionlib.SimpleActionClient(action_ns,
                                                    GripperCommandAction)
        self._client.wait_for_server()
        self._goal    = GripperCommandGoal()
        self._min_gap = min_gap
        self._max_gap = max_gap

        self.parameters = {'max_effort':       max_effort,
                           'grasp_position':   self._min_gap,
                           'release_position': self._max_gap}

        rospy.loginfo('{} initialized.'.format(self.name))

    @staticmethod
    def base(name, action_ns, base_link, tip_link, timeout,
             min_gap, max_gap, max_effort):
        return GenericGripper(name, action_ns, base_link, tip_link, timeout,
                              min_gap, max_gap, max_effort)

    def pregrasp(self):
        return self.release(cmd)

    def grasp(self):
        return self.move(self.parameters['grasp_position'])

    def release(self):
        return self.move(self.parameters['release_position'])

    def move(self, position):
        self._goal.command.max_effort = self.parameters['max_effort']
        self._goal.command.position   = clip(position,
                                             self._min_gap, self._max_gap)
        self._client.send_goal(self._goal)
        if not self._client.wait_for_result(rospy.Duration(self.timeout)):
            rospy.logerr('Timeout[%f] has expired before goal finished',
                         self.timeout)
            return False
        result = self._client.get_result()
        rospy.loginfo(result)
        return result.reached_goal or result.stalled

######################################################################
#  class RobotiqGripper                                              #
######################################################################
class RobotiqGripper(GripperClient):
    def __init__(self, prefix='a_bot_', product='robotiq_85',
                 force=5.0, velocity=0.1, timeout=6.0):
        import robotiq_msgs.msg

        super(RobotiqGripper, self) \
            .__init__(*RobotiqGripper._initargs(prefix, product,
                                                force, velocity, timeout))
        self._client = actionlib.SimpleActionClient(
                           prefix
                             + 'gripper_controller/gripper_action_controller',
                           robotiq_msgs.msg.CModelCommandAction)
        self._goal   = robotiq_msgs.msg.CModelCommandGoal()

        if product == 'robotiq_hande':
            self._min_gap   = 0.0
            self._max_gap   = 0.026
            self._min_speed = 0.02
            self._max_speed = 0.15
            self._min_force = 20.0
            self._max_force = 130.0
        elif product == 'robotiq_85':
            self._min_gap   = 0.0
            self._max_gap   = 0.085
            self._min_speed = 0.013
            self._max_speed = 0.1
            self._min_force = 20.0
            self._max_force = 235.0
        else:
            self._min_gap   = 0.0
            self._max_gap   = 0.140
            self._min_speed = 0.03
            self._max_speed = 0.25
            self._min_force = 10.0
            self._max_force = 125.0

        self.parameters = {'max_effort':       force,
                           'velocity':         velocity,
                           'grasp_position':   self._min_gap,
                           'release_position': self._max_gap}

    @staticmethod
    def base(prefix, product, force, velocity, timeout):
        return GripperClient(*RobotiqGripper._initargs(prefix, product,
                                                       force, velocity,
                                                       timeout))
        # return RobotiqGripper(prefix, product, force, velocity, timeout)

    @staticmethod
    def _initargs(prefix, product, force, velocity, timeout):
        return (prefix + product + '_gripper', 'two_finger',
                prefix + product + '_base_link',
                prefix + product + '_tip_link', timeout)

    def pregrasp(self):
        return self.release()

    def grasp(self):
        return self.move(self.parameters['grasp_position'])

    def release(self):
        return self.move(self.parameters['release_position'])

    def move(self, position):
        try:
            self._goal.force    = clip(self.parameters['max_effort'],
                                       self._min_force, self._max_force)
            self._goal.velocity = clip(self.parameters['velocity'],
                                       self._min_speed, self._max_speed)
            self._goal.position = clip(position, self._min_gap, self._max_gap)
            self._client.send_goal(self._goal)
            # This sleep is necessary for robotiq gripper to work just as intended.
            rospy.sleep(.5)
            if not self._client.wait_for_result(rospy.Duration(self.timeout)):
                rospy.logerr('Timeout[%f] has expired before goal finished',
                             self.timeout)
                return False
            result = self._client.get_result()
            return result.reached_goal
        except rospy.ROSInterruptException:
            rospy.loginfo(
                'RobotiqGripper: program interrupted before completion.',
                file=sys.stderr)

######################################################################
#  class SuctionGripper                                              #
######################################################################
class SuctionGripper(GripperClient):
    def __init__(self, name, base_link, tip_link, eject=False, timeout=2.0):
        import o2as_msgs.msg

        super(SuctionGripper, self) \
            .__init__(*SuctionGripper._initargs(name, base_link, tip_link,
                                                eject, timeout))
        self._client    = actionlib.SimpleActionClient(
                              'o2as_fastening_tools/suction_control',
                              o2as_msgs.msg.SuctionControlAction)
        self._sub       = rospy.Subscriber('suction_tool/screw_suctioned',
                                           std_msgs.msg.Bool,
                                           self._state_callback)
        self._suctioned = False
        self._eject     = eject  # blow when releasing
        self._goal                     = o2as_msgs.msg.SuctionControlGoal()
        self._goal.fastening_tool_name = 'suction_tool'
        self._goal.turn_suction_on     = False
        self._goal.eject_screw         = False

    @staticmethod
    def base(name, base_link, tip_link, eject, timeout):
        return GripperClient(*SuctionGripper._initargs(name,
                                                       base_link, tip_link,
                                                       eject, timeout))

    @staticmethod
    def _initargs(name, base_link, tip_link, eject, timeout):
        return (name, 'suction', base_link, tip_link, timeout)

    @property
    def suctioned(self):
        return self._suctioned

    def pregrasp(self):
        return self._send_command(True)

    def grasp(self):
        if not self._send_command(True):
            return False
        rospy.sleep(0.5)        # Wait until the state is updated.
        return self._suctioned

    def release(self):
        return self._send_command(False)

    def _send_command(self, turn_suction_on):
        try:
            self._goal.turn_suction_on = turn_suction_on
            self._goal.eject_screw     = not turn_suction_on and self._eject
            self._client.send_goal(self._goal)
            if not self._client.wait_for_result(rospy.Duration(self.timeout)):
                rospy.logerr('Timeout[%f] has expired before goal finished',
                             self.timeout)
                return False
            result = self._client.get_result()
            return result.success
        except rospy.ROSInterruptException:
            rospy.loginfo(
                'SuctionGripper: program interrupted before completion.',
                file=sys.stderr)
            return False

    def _state_callback(self, msg):
        self._suctioned = msg.data

######################################################################
#  class PrecisionGripper                                            #
######################################################################
class PrecisionGripper(GripperClient):
    def __init__(self, prefix='a_bot_', timeout=3.0):
        import o2as_msgs.msg

        super(PrecisionGripper, self) \
            .__init__(*PrecisionGripper._initargs(prefix, timeout))
        self._client = actionlib.SimpleActionClient(
                           'precision_gripper_action',
                           # str(prefix) + 'gripper/gripper_action_controller',
                           o2as_msgs.msg.PrecisionGripperCommandAction)
        self._goal = o2as_msgs.msg.PrecisionGripperCommandGoal()
        self._goal.stop                         = False
        self._goal.open_outer_gripper_fully     = False
        self._goal.close_outer_gripper_fully    = False
        self._goal.open_inner_gripper_fully     = False
        self._goal.close_inner_gripper_fully    = False
        self._goal.this_action_grasps_an_object = False
        self._goal.linear_motor_position        = 0.0
        self._goal.outer_gripper_opening_width  = 0.0
        self._goal.inner_gripper_opening_width  = 0.0
        self._goal.slight_opening_width         = 0.0

        self.parameters = {'cmd': ''}

    @staticmethod
    def base(prefix, timeout):
        return GripperClient(*PrecisionGripper._initargs(prefix, timeout))

    @staticmethod
    def _initargs(prefix, timeout):
        return (prefix + 'gripper', 'two_finger',
                prefix + 'gripper_base_link',
                prefix + 'gripper_tip_link', timeout)

    @property
    def linear_motor_position(self):
        return self._goal.linear_motor_position

    def pregrasp(self):
        cmd = self.parameters['cmd']
        if cmd == 'complex_pick_from_inside':
            self._inner_command(True)
        elif cmd == 'complex_pick_from_outside':
            self._inner_command(False)
        elif cmd == 'easy_pick_only_inner' or \
             cmd == 'inner_gripper_from_inside' or \
             cmd == '':
            self._inner_command(False)
        elif cmd == 'easy_pick_outside_only_inner' or \
             cmd == 'inner_gripper_from_outside':
            self._inner_command(True)
        else:
            return False
        return True

    def grasp(self):
        cmd = self.parameters['cmd']
        if cmd == 'complex_pick_from_inside':
            self._inner_command(False, True)
            self._outer_command(True)
        elif cmd == 'complex_pick_from_outside':
            self._inner_command(True, True)
            self._outer_command(True)
        elif cmd == 'easy_pick_only_inner' or \
             cmd == 'inner_gripper_from_inside' or \
             cmd == '':
            self._inner_command(True, True)
        elif cmd == 'easy_pick_outside_only_inner' or \
             cmd == 'inner_gripper_from_outside':
            self._inner_command(False, True)
        else:
            return False
        return True

    def release(self):
        cmd = self.parameters['cmd']
        if cmd == 'complex_pick_from_inside':
            self._outer_command(False)
            self._inner_command(True)
        elif cmd == 'complex_pick_from_outside':
            self._outer_command(False)
            self._inner_command(False)
        elif cmd == 'easy_pick_only_inner' or \
             cmd == 'inner_gripper_from_inside' or \
             cmd == '':
            self._inner_command(False)
        elif cmd == 'easy_pick_outside_only_inner' or \
             cmd == 'inner_gripper_from_outside':
            self._inner_command(True)
        else:
            return False
        return True

    def _inner_command(self, close, grasps_an_object=False):
        try:
            self._goal.open_inner_gripper_fully     = not close
            self._goal.close_inner_gripper_fully    = close
            self._goal.open_outer_gripper_fully     = False
            self._goal.close_outer_gripper_fully    = False
            self._goal.this_action_grasps_an_object = grasps_an_object
            self._client.send_goal(self._goal)
            if not self._client.wait_for_result(rospy.Duration(self.timeout)):
                rospy.logerr('Timeout[%f] has expired before goal finished',
                             self.timeout)
                return
            result = self._client.get_result()
            rospy.loginfo(result)
        except rospy.ROSInterruptException:
            rospy.loginfo(
                'PrecisionGripper: program interrupted before completion.',
                file=sys.stderr)

    def _outer_command(self, close, grasps_an_object=False):
        try:
            self._goal.open_inner_gripper_fully     = False
            self._goal.close_inner_gripper_fully    = False
            self._goal.open_outer_gripper_fully     = not close
            self._goal.close_outer_gripper_fully    = close
            self._goal.this_action_grasps_an_object = grasps_an_object
            self._client.send_goal(self._goal)
            if not self._client.wait_for_result(rospy.Duration(self.timeout)):
                rospy.logerr('Timeout[%f] has expired before goal finished',
                             self.timeout)
                return
            result = self._client.get_result()
            rospy.loginfo(result)
        except rospy.ROSInterruptException:
            rospy.loginfo(
                'PrecisionGripper: program interrupted before completion.',
                file=sys.stderr)

######################################################################
#  class Lecp6Gripper                                                #
######################################################################
class Lecp6Gripper(GripperClient):
    def __init__(self, prefix='a_bot_', timeout=3.0, open_no=1, close_no=2):
        import tranbo_control.msg

        super(Lecp6Gripper, self) \
            .__init__(*Lecp6Gripper._initargs(prefix, timeout))
        self._client = actionlib.SimpleActionClient(
                           '/arm_driver/lecp6_driver/lecp6',
                           tranbo_control.msg.Lecp6CommandAction)
        self._goal = tranbo_control.msg.Lecp6CommandGoal()
        self._picked = False

        self.parameters = {'release_stepdata': open_no,
                           'grasp_stepdata':   close_no}

    @staticmethod
    def base(prefix, timeout):
        return GripperClient(*Lecp6Gripper._initargs(prefix, timeout))

    @staticmethod
    def _initargs(prefix, timeout):
        return (prefix + 'gripper', 'two_finger',
                prefix + 'gripper_base_link',
                prefix + 'gripper_link', timeout)

    @property
    def picked(self):
        return self._picked

    def pregrasp(self):
        return self.release()

    def grasp(self):
        return self._send_command(True)

    def release(self):
        return self._send_command(False)

    def _send_command(self, close):
        try:
            self._picked = False
            self._goal.command.stepdata_no \
                = self.parameters['grasp_stepdata' if close else
                                  'release_stepdata']
            self._client.send_goal(self._goal)
            if not self._client.wait_for_result(rospy.Duration(self.timeout)):
                rospy.logerr('Timeout[%f] has expired before goal finished',
                             self.timeout)
                return False
            result = self._client.get_result()
            if close:
                self._picked = result.reached_goal
            rospy.loginfo(result)
            return result.reached_goal
        except rospy.ROSInterruptException:
            rospy.loginfo(
                'Lecp6Gripper: program interrupted before completion.',
                file=sys.stderr)
            return False

######################################################################
#  class MagswitchGripper                                            #
######################################################################
class MagswitchGripper(GripperClient):
    def __init__(self, prefix='a_bot_', timeout=5.0,
                 sensitivity=0, position=100):
        import tranbo_control.msg

        super(MagswitchGripper, self) \
            .__init__(*MagswitchGripper._initargs(prefix, timeout))
        self._client = actionlib.SimpleActionClient(
                              '/arm_driver/magswitch_driver/magswitch',
                              tranbo_control.msg.MagswitchCommandAction)
        self._goal   = tranbo_control.msg.MagswitchCommandGoal()

        self._calibration_step = 0
        self._suctioned        = False

        self.parameters = {'sensitivity':    sensitivity,
                           'grasp_position': position}

    @staticmethod
    def base(prefix, timeout):
        return GripperClient(*MagswitchGripper._initargs(prefix, timeout))

    @staticmethod
    def _initargs(prefix, timeout):
        return (prefix + 'magnet', 'suction',
                prefix + 'magnet_base_link',
                prefix + 'magnet_link', timeout)

    @property
    def suctioned(self):
        return self._suctioned

    @property
    def calibration_step(self):
        return self._calibration_step

    def calibration(self, calibration_select):
        return self._send_command(calibration_trigger=1,
                                  calibration_select=calibration_select)

    def pregrasp(self):
        return self.grasp()

    def grasp(self):
        return self._send_command(position=self.parameters['grasp_position'])

    def postgrasp(self):
        return self._send_command(position=100)

    def release(self):
        return self._send_command(position=0)

    def _send_command(self,
                      calibration_trigger=0, calibration_select=0, position=0):
        try:
            self._calibration_step = 0
            self._goal.used_sdo = False
            self._goal.command.calibration_trigger = calibration_trigger
            self._goal.command.calibration_select  = calibration_select
            self._goal.command.sensitivity \
                = clip(self.parameters['sensitivity'], -30, 30)
            self._goal.command.position            = clip(position, 0, 100)
            self._client.send_goal(self._goal)
            if not self._client.wait_for_result(rospy.Duration(self.timeout)):
                rospy.logerr('Timeout[%f] has expired before goal finished',
                             self.timeout)
                return False
            result = self._client.get_result()
            if calibration_trigger == 1:
                self._calibration_step = result.magswitch_out.calibration_step
            self._suctioned = position > 0 and \
                              result.magswitch_out.calibration_state != 0
            return result.reached_goal

        except rospy.ROSInterruptException:
            rospy.loginfo(
                'MagswitchGripper: program interrupted before completion.',
                file=sys.stderr)
            return False
