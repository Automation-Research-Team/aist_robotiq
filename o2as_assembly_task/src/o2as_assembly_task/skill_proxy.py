import rospy
from util import *
from std_srvs.srv import *
from o2as_msgs.srv import *
from o2as_vision.srv import *
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point
import actionlib
import o2as_msgs.msg

FIND_OBJECT_SERVICE = "find_object"

class SkillProxy(object):
    def __init__(self):
        # vision
        self._find_object = ros_service_proxy(FIND_OBJECT_SERVICE, FindObject)

        # skills
        # self._pick_action = actionlib.SimpleActionClient('pick_action', o2as_msgs.msg.pickAction)
        # self._pick_action.wait_for_server()
        rospy.sleep(.5)   # Use this instead of waiting, so that simulation can be used

    def find_object(self, expected_position, position_tolerance, object_id, camera):
        rospy.loginfo("SkillProxy.find_object() begin")
        rospy.loginfo("expected_position: ")
        rospy.loginfo(expected_position.pose.position)
        rospy.loginfo("position_tolerance = " + str(position_tolerance))
        rospy.loginfo("object_id = " + str(object_id))
        rospy.loginfo("camera = " + str(camera))

        try:
            res = self._find_object(expected_position, position_tolerance, object_id, camera)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))

    def pick(self, robot_name, item_id="", item_pose=None):
        rospy.logerr("SkillProxy.pick() is not implemented")
        # try:
        #     goal = o2as_msgs.msg.pickGoal()
        #     goal.use_complex_planning = False   # Use simple grasp planning from above or complex planning?
        #     goal.gripper_command = "inner_only" # "inner_only", "complex_pick_from_inside", "complex_pick_from_outside"
        #     goal.retreat_after_pick = True      # If true, arm moves away from the table after grasping
        #     goal.item_id = ""                   # item ID in the planning scene
        #     goal.item_pose = PoseStamped()      # This overrides the item_id
        #     goal.robot_name = "b_bot"           # "a_bot", "b_bot", "c_bot"
        #     goal.tool_name = ""                 # "outer_gripper", "inner_gripper", "suction", "screw_tool" or empty
        #     goal.screw_size = 4                 # M4, M5, M6 (without the M)

        #     self._pick_action.send_goal(goal)
        #     rospy.loginfo("pick parts")
        #     self._pick_action.wait_for_result()
        #     result = self._pick_action.get_result()
        #     rospy.loginfo(result)
        # except rospy.ROSInterruptException:
        #     rospy.loginfo("program interrupted before completion", file=sys.stderr)

    def pick(self, robotname, object_pose, grasp_height, speed_fast, speed_slow, gripper_command="", approach_height = 0.03):
        pass
        # #initial gripper_setup
        # rospy.loginfo("Going above object to pick")
        # object_pose.pose.position.z = approach_height
        # self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

        # # if gripper_command=="complex_pick_from_inside":
        # #     self.precision_gripper_inner_close() 
        # # elif gripper_command=="complex_pick_from_outside":
        # #     self.precision_gripper_inner_open()
        # # elif gripper_command=="easy_pick_only_inner":
        # #     self.precision_gripper_inner_close()
        # # else: 
        # #     rospy.logerr("No gripper command was set")

        # rospy.loginfo("Moving down to object")
        # object_pose.pose.position.z = grasp_height
        # rospy.loginfo(grasp_height)
        # self.go_to_pose_goal(robotname, object_pose, speed=speed_slow, high_precision=True)

        # # W = raw_input("waiting for the gripper")
        # # #gripper close
        # # if gripper_command=="complex_pick_from_inside":
        # #     self.precision_gripper_inner_open(this_action_grasps_an_object = True)
        # #     self.precision_gripper_outer_close()
        # # elif gripper_command=="complex_pick_from_outside":
        # #     self.precision_gripper_inner_close(this_action_grasps_an_object = True)
        # #     self.precision_gripper_outer_close()
        # # elif gripper_command=="easy_pick_only_inner":
        # #     self.precision_gripper_inner_open(this_action_grasps_an_object = True)
        # rospy.sleep(2)
        # rospy.loginfo("Going back up")
        # object_pose.pose.position.z = (approach_height)
        # self.go_to_pose_goal(robotname, object_pose, speed=speed_fast)

    def insert(self, robot_name):
        rospy.logerr("SkillProxy.insert() is not implemented")
