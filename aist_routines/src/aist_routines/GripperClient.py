import rospy
import actionlib
import std_msgs.msg

from numpy import clip

######################################################################
#  class GripperClient                                               #
######################################################################
class GripperClient(object):
    def __init__(self, name, type, base_link, tip_link, timeout):
        self._name      = name
        self._type      = type
        self._base_link = base_link
        self._tip_link  = tip_link
        self._timeout   = timeout

    @staticmethod
    def create(type_name, kwargs):
        ClientClass = globals()[type_name]
        if rospy.get_param("use_real_robot", False):
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

    @timeout.setter
    def timeout(self, t):
        self._timeout = t

    def pregrasp(self, cmd=""):
        return True

    def grasp(self, cmd=""):
        return True

    def release(self, cmd=""):
        return True

######################################################################
#  class RobotiqGripper                                              #
######################################################################
class RobotiqGripper(GripperClient):
    def __init__(self, prefix="a_bot_", product="85",
                 force=5.0, velocity=0.1, timeout=6.0):
        import robotiq_msgs.msg

        super(RobotiqGripper, self) \
            .__init__(*RobotiqGripper._initargs(prefix, product,
                                                force, velocity, timeout))
        if product == "hande":
            self._min_gap   = 0.0
            self._max_gap   = 0.026
            self._min_speed = 0.02
            self._max_speed = 0.15
            self._min_force = 20.0
            self._max_force = 130.0
        elif product == "85":
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
        self._open_position = self._max_gap

        self._client = actionlib.SimpleActionClient(
                           str(prefix) + "gripper/gripper_action_controller",
                           robotiq_msgs.msg.CModelCommandAction)
        self._goal          = robotiq_msgs.msg.CModelCommandGoal()
        self._goal.force    = clip(force, self._min_force, self._max_force)
        self._goal.velocity = clip(velocity, self._min_speed, self._max_speed)
        self._goal.position = 0.0

    @staticmethod
    def base(prefix, product, force, velocity, timeout):
        return GripperClient(*RobotiqGripper._initargs(prefix, product,
                                                       force, velocity,
                                                       timeout))

    @staticmethod
    def _initargs(prefix, product, force, velocity, timeout):
        return (prefix + "robotiq_" + product + "_gripper", "two_finger",
                prefix + "robotiq_" + product + "_base_link",
                prefix + "robotiq_" + product + "_tip_link", timeout)

    @property
    def open_position(self):
        return self._open_position

    @open_position.setter
    def open_position(self, position):
        self._open_position = clip(position, self._min_gap, self._max_gap)

    @property
    def max_effort(self):
        return self._goal.force

    @max_effort.setter
    def max_effort(self, force):
        self._goal.force = clip(force, self._min_force, self._max_force)

    @property
    def velocity(self):
        return self._goal.velocity

    @velocity.setter
    def velocity(self, v):
        self._goal.velocity = clip(v, self._min_speed, self._max_speed)

    def pregrasp(self, cmd=""):
        return self.release()

    def grasp(self, cmd=""):
        return self.move(self._min_gap)

    def release(self, cmd=""):
        return self.move(self._open_position)

    def move(self, position):
        try:
            self._goal.position = clip(position, self._min_gap, self._max_gap)
            self._client.send_goal(self._goal)
            # This sleep is necessary for robotiq gripper to work just as intended.
            rospy.sleep(.5)
            self._client.wait_for_result(rospy.Duration(self.timeout))
            result = self._client.get_result()
            return result.reached_goal
        except rospy.ROSInterruptException:
            rospy.loginfo(
                "RobotiqGripper: program interrupted before completion.",
                file=sys.stderr)

######################################################################
#  class FetchGripper                                                #
######################################################################
class FetchGripper(GripperClient):
    def __init__(self, prefix="a_fetch_", max_effort=5.0, timeout=10.0):
        from control_msgs.msg import GripperCommandAction, GripperCommandGoal

        super(FetchGripper, self) \
            .__init__(prefix + "gripper", "two_finger",
                      prefix + "wrist_roll_link",
                      prefix + "gripper_link", timeout)
        self._client = actionlib.SimpleActionClient(
                           prefix + "gripper_controller/gripper_action",
                           GripperCommandAction)
        self._client.wait_for_server()
        self._goal = GripperCommandGoal()
        self._goal.command.max_effort = max_effort
        self._open_position = 0.1
        rospy.loginfo("{} initialized.".format(self.name))

    @staticmethod
    def base(prefix, max_effort, timeout):
        return FetchGripper(prefix, max_effort, timeout)

    @property
    def open_position(self):
        return self._open_position

    @open_position.setter
    def open_position(self, position):
        self._open_position = clip(position, 0.0, 0.1)

    @property
    def max_effort(self):
        return self._goal.command.max_effort

    @max_effort.setter
    def max_effort(self, max_effort):
        self._goal.command.max_effort = max_effort

    def pregrasp(self, cmd=""):
        return self.release(cmd)

    def grasp(self, cmd=""):
        return self.move(0.0)

    def release(self, cmd=""):
        return self.move(self._open_position)

    def move(self, position):
        self._goal.command.position = clip(position, 0.0, 0.1)
        self._client.send_goal(self._goal)
        if self._client.wait_for_result(rospy.Duration(self.timeout)):
            result = self._client.get_result()
            if result:
                rospy.loginfo(result)
                return result.reached_goal or result.stalled
            else:
                rospy.logerr("No goal is running.")
                return False
        else:
            rospy.logerr("No goal exists.")
            return False

######################################################################
#  class ToroboGripper                                               #
######################################################################
class ToroboGripper(GripperClient):
    def __init__(self, prefix="a_torobo_", max_effort=5.0):
        from control_msgs.msg import GripperCommandAction, GripperCommandGoal

        super(ToroboGripper, self) \
            .__init__(prefix + "gripper", "two_finger",
                      prefix + "gripper/base_link",
                      prefix + "gripper/tip_link")
        self._client = actionlib.SimpleActionClient(
            "/torobo/" + prefix + "gripper_controller/gripper_cmd",
            GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(10.0))
        self._goal = GripperCommandGoal()
        self._goal.command.max_effort = max_effort
        self._open_position = 0.1
        rospy.loginfo("{} initialized.".format(self.name))

    @staticmethod
    def base(prefix, max_effort):
        return ToroboGripper(prefix, max_effort)

    @property
    def open_position(self):
        return self._open_position

    @open_position.setter
    def open_position(self, position):
        self._open_position = clip(position, 0.0, 0.1)

    @property
    def max_effort(self):
        return self._goal.command.max_effort

    @max_effort.setter
    def max_effort(self, max_effort):
        self._goal.command.max_effort = max_effort

    def pregrasp(self, cmd=""):
        return self.release(cmd)

    def grasp(self, cmd=""):
        return self.move(0.0)

    def release(self, cmd=""):
        return self.move(self._open_position)

    def move(self, position):
        try:
            self._goal.command.position = clip(position, 0.0, 0.1)
            self._client.send_goal(self._goal)
            self._client.wait_for_result(rospy.Duration(5.0))
            result = self._client.get_result()
            return result.reached_goal
        except rospy.ROSInterruptException:
            rospy.logerr('Gripper: program interrupted before completion.')

######################################################################
#  class SuctionGripper                                              #
######################################################################
class SuctionGripper(GripperClient):
    def __init__(self, prefix="b_bot_single_", eject=False, timeout=2.0):
        import o2as_msgs.msg

        super(SuctionGripper, self) \
            .__init__(*SuctionGripper._initargs(prefix, eject))
        self._client    = actionlib.SimpleActionClient(
                              "o2as_fastening_tools/suction_control",
                              o2as_msgs.msg.SuctionControlAction)
        self._sub       = rospy.Subscriber("suction_tool/screw_suctioned",
                                           std_msgs.msg.Bool,
                                           self._state_callback)
        self._suctioned = False
        self._eject     = eject  # blow when releasing
        self._goal                     = o2as_msgs.msg.SuctionControlGoal()
        self._goal.fastening_tool_name = "suction_tool"
        self._goal.turn_suction_on     = False
        self._goal.eject_screw         = False

    @staticmethod
    def base(prefix, eject, timeout):
        return GripperClient(*SuctionGripper._initargs(prefix, eject, timeout))

    @staticmethod
    def _initargs(prefix, eject, timeout):
        return (prefix + "suction_gripper", "suction",
                prefix + "suction_gripper_base_link",
                prefix + "suction_gripper_pad_link", timeout)

    @property
    def eject(self):
        return self._eject

    @eject.setter
    def eject(self, e):
        self._eject = e

    @property
    def suctioned(self):
        return self._suctioned

    def pregrasp(self, cmd=""):
        return self._send_command(True)

    def grasp(self, cmd=""):
        if not self._send_command(True):
            return False
        rospy.sleep(0.5)        # Wait until the state is updated.
        return self._suctioned

    def release(self, cmd=""):
        return self._send_command(False)

    def _send_command(self, turn_suction_on):
        try:
            self._goal.turn_suction_on = turn_suction_on
            self._goal.eject_screw     = not turn_suction_on and self._eject
            self._client.send_goal(self._goal)
            self._client.wait_for_result(rospy.Duration(self.timeout))
            result = self._client.get_result()
            return result.success
        except rospy.ROSInterruptException:
            rospy.loginfo(
                "SuctionGripper: program interrupted before completion.",
                file=sys.stderr)
            return False

    def _state_callback(self, msg):
        self._suctioned = msg.data

######################################################################
#  class PrecisionGripper                                            #
######################################################################
class PrecisionGripper(GripperClient):
    def __init__(self, prefix="a_bot_", timeout=3.0):
        import o2as_msgs.msg

        super(PrecisionGripper, self) \
            .__init__(*PrecisionGripper._initargs(prefix, timeout))
        self._client = actionlib.SimpleActionClient(
                           "precision_gripper_action",
                           # str(prefix) + "gripper/gripper_action_controller",
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

    @staticmethod
    def base(prefix, timeout):
        return GripperClient(*PrecisionGripper._initargs(prefix, timeout))

    @staticmethod
    def _initargs(prefix, timeout):
        return (prefix + "gripper", "two_finger",
                prefix + "gripper_base_link",
                prefix + "gripper_tip_link", timeout)

    @property
    def linear_motor_position(self):
        return self._goal.linear_motor_position

    def pregrasp(self, cmd=""):
        if cmd == "complex_pick_from_inside":
            self._inner_command(True)
        elif cmd == "complex_pick_from_outside":
            self._inner_command(False)
        elif cmd == "easy_pick_only_inner" or \
             cmd == "inner_gripper_from_inside" or \
             cmd == "":
            self._inner_command(False)
        elif cmd == "easy_pick_outside_only_inner" or \
             cmd == "inner_gripper_from_outside":
            self._inner_command(True)
        else:
            return False
        return True

    def grasp(self, cmd=""):
        if cmd == "complex_pick_from_inside":
            self._inner_command(False, True)
            self._outer_command(True)
        elif cmd == "complex_pick_from_outside":
            self._inner_command(True, True)
            self._outer_command(True)
        elif cmd == "easy_pick_only_inner" or \
             cmd == "inner_gripper_from_inside" or \
             cmd == "":
            self._inner_command(True, True)
        elif cmd == "easy_pick_outside_only_inner" or \
             cmd == "inner_gripper_from_outside":
            self._inner_command(False, True)
        else:
            return False
        return True

    def release(self, cmd=""):
        if cmd == "complex_pick_from_inside":
            self._outer_command(False)
            self._inner_command(True)
        elif cmd == "complex_pick_from_outside":
            self._outer_command(False)
            self._inner_command(False)
        elif cmd == "easy_pick_only_inner" or \
             cmd == "inner_gripper_from_inside" or \
             cmd == "":
            self._inner_command(False)
        elif cmd == "easy_pick_outside_only_inner" or \
             cmd == "inner_gripper_from_outside":
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
            self._client.wait_for_result(rospy.Duration(self.timeout))
            result = self._client.get_result()
            rospy.loginfo(result)
        except rospy.ROSInterruptException:
            rospy.loginfo(
                "PrecisionGripper: program interrupted before completion.",
                file=sys.stderr)

    def _outer_command(self, close, grasps_an_object=False):
        try:
            self._goal.open_inner_gripper_fully     = False
            self._goal.close_inner_gripper_fully    = False
            self._goal.open_outer_gripper_fully     = not close
            self._goal.close_outer_gripper_fully    = close
            self._goal.this_action_grasps_an_object = grasps_an_object
            self._client.send_goal(self._goal)
            self._client.wait_for_result(rospy.Duration(self.timeout))
            result = self._client.get_result()
            rospy.loginfo(result)
        except rospy.ROSInterruptException:
            rospy.loginfo(
                "PrecisionGripper: program interrupted before completion.",
                file=sys.stderr)
