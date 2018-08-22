import rospy
from parts_info import *
from skill_proxy import SkillProxy
from geometry_msgs.msg import PoseStamped

class PipelineTest(object):
    def __init__(self):
        self.init_parts_dict()
        self._proxy = SkillProxy()

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

    def pick(self):
        # find parts
        expected_position   = PoseStamped()     # belt room center
        position_tolerance  = 0.2               # within 20 cm
        object_id           = "11"              # output-shaft-pulley
        camera              = "b_bot_camera"    # should be changed to phoxi

        while not rospy.core.is_shutdown():
            item_pose = self._proxy.find_object(expected_position, position_tolerance, object_id, camera)
            if item_pose is not None:
                break

        # pick parts
        self._proxy.pick(robotname="b_bot", object_pose=item_pose, grasp_height=0.2, 
            approach_height=0.05, speed_fast=0.2, speed_slow=0.02, gripper_command="")

        # pick parts
        self._proxy.pick(robot_name="", item_pose=item_pose)

    def run(self, product_count = 1):
        test = PipelineTest()
        test.pick()
        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)
