#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

LOG_LEVEL = log_level=rospy.INFO

class BBotVisionTest(object):
    def __init__(self):
        self.init_parts_dict()
        self._proxy = SkillProxy()

    def init_parts_dict(self):
        self._parts_dict = dict()
        parts_list = rospy.get_param("~parts_list")
        for item in parts_list:
            self._parts_dict[item['name']] = item
            rospy.logdebug(item)

    def pick(self):
        rospy.loginfo("Will look for item 11 (output-shaft-pulley) and try to pick it up. Press enter to proceed.")
        raw_input()

        # find parts
        expected_position   = PoseStamped()     # belt room center
        position_tolerance  = 0.2               # within 20 cm
        object_id           = "11"              # output-shaft-pulley
        camera              = "b_bot_camera"    # should be changed to phoxi

        while not rospy.core.is_shutdown():
            item_pose = self._proxy.find_object(expected_position, position_tolerance, object_id, camera)
            if item_pose is not None:
                rospy.loginfo("Found the object at pose:")
                rospy.loginfo(item_pose)
                break

        # pick parts
        self._proxy.pick(robotname="b_bot", object_pose=item_pose, grasp_height=0.2, 
            approach_height=0.05, speed_fast=0.2, speed_slow=0.02, gripper_command="")

    def run(self, product_count = 1):
        test = BBotVisionTest()
        test.pick()
        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)

if __name__ == "__main__":
    rospy.init_node('o2as_assembly_task', anonymous=True, log_level=LOG_LEVEL)
    test = PipelineTest()
    test.run()
