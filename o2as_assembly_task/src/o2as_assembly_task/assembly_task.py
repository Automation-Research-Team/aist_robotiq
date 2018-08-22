import rospy
from util import *
from parts_info import *
from subtask import SubTask

from std_srvs.srv import *
from o2as_msgs.srv import *
from o2as_vision.srv import *
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point
import actionlib
import o2as_msgs.msg

FIND_OBJECT_SERVICE = "find_object"

class AssemblyTask(object):
    def __init__(self):
        self.init_subtasks()
        self.init_parts_dict()

        # vision
        self._find_object = ros_service_proxy(FIND_OBJECT_SERVICE, FindObject)

        # skills
        self._pick_action = actionlib.SimpleActionClient('pick_action', o2as_msgs.msg.pickAction)
        # self._pick_action.wait_for_server()
        rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used


    #############
    ### parts ###
    #############

    def init_parts_dict(self):
        self._parts_name_dict = dict()

        # initialize dict using yaml file
        belt_drive_unit_parts = rospy.get_param("~belt_drive_unit_parts")
        for item in belt_drive_unit_parts:
            name = item.keys()[0]
            info = PartsInfo(name=name, id=item[name]['id'], type=item[name]['type'], desc=item[name]['description'])
            self._parts_name_dict[name] = info
            rospy.logdebug("parts name=%s, id=%s, type=%s, desc=%s", name, info.id, info.type, info.desc)

    def get_parts_info(self, name):
        return self._parts_name_dict[name]

    ##############
    ### skills ###
    ##############

    def pick(self):
        try:
            goal = o2as_msgs.msg.pickGoal()
            goal.use_complex_planning = False   # Use simple grasp planning from above or complex planning?
            goal.gripper_command = "inner_only" # "inner_only", "complex_pick_from_inside", "complex_pick_from_outside"
            goal.retreat_after_pick = True      # If true, arm moves away from the table after grasping
            goal.item_id = ""                   # item ID in the planning scene
            goal.item_pose = PoseStamped()      # This overrides the item_id
            goal.robot_name = "b_bot"           # "a_bot", "b_bot", "c_bot"
            goal.tool_name = ""                 # "outer_gripper", "inner_gripper", "suction", "screw_tool" or empty
            goal.screw_size = 4                 # M4, M5, M6 (without the M)

            self._pick_action.send_goal(goal)
            rospy.loginfo("pick parts")
            self._pick_action.wait_for_result()
            result = self._pick_action.get_result()
            rospy.loginfo(result)
        except rospy.ROSInterruptException:
            rospy.loginfo("program interrupted before completion", file=sys.stderr)

    def find_object(self, expected_position, position_tolerance, object_id, camera):
        try:
            res = self._find_object(expected_position, position_tolerance, object_id, camera)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e))

    ################
    ### subtasks ###
    ################

    def init_subtasks(self):
        self._subtasks = []
        self._subtasks = [ SubTask("dummy_subtask", self.dummy_subtask, self) ]

    # Assembly Parts
    #  5: motor_pulley
    #  7: bearing_housing
    #  8: drive_shaft
    # 11: output_pulley
    def dummy_subtask(self, **kwargs):
        rospy.logdebug("AssemblyTask.dummy_subtask() begin")

        # this dummy subtask just find object and add it to planning scene for the moment
        rospy.logdebug("find parts")

        # find object
        expected_position = PoseStamped()
        expected_position.header = Header(stamp=rospy.Time.now(), frame_id='world')
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose = Pose()
        pose.position = Point(x=0.0, y=0.0, z=0.0)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        position_tolerance = 0.2 # 20cm
        parts_info = self.get_parts_info("output_pulley")
        object_id = str(parts_info.id)
        camera = "b_bot_camera"
        rospy.logdebug("find_object()) object_id=%s, type=%s", parts_info.id, parts_info.type)
        self.find_object(expected_position, position_tolerance, object_id, camera)

        # pick object
        self.pick()

    ############
    ### main ###
    ############

    def assemble_belt_unit(self):
        # execute all subtasks
        for subtask in self._subtasks:
            subtask.run()
            # TODO: monitor execution states of subtask
            # TODO: judge execution result of subtask

    def run(self, product_count = 1):
        # assemble product
        for i in range(product_count):
            self.assemble_belt_unit()

        # loop
        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)
