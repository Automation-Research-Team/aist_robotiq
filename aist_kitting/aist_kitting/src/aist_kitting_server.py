#!/usr/bin/env python

import copy
import sys
import os

import rospy
from aist_kitting_msgs.srv import *
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseStamped
from o2as_phoxi_camera.srv import SetInt

from cv_bridge import CvBridge, CvBridgeError
import cv2
from skimage import io
import moveit_commander
import moveit_msgs
from moveit_commander.conversions import pose_to_list


class AistKitting:
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        rospy.wait_for_service('o2as_phoxi_camera/start_acquisition')

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        group_name = rospy.get_param("move_group_name", "a_bot")
        rospy.loginfo(group_name)
        eef_link = rospy.get_param("ee_link", "a_bot_dual_suction_gripper_pad_link")
        rospy.loginfo(eef_link)
        group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # Misc variables
        self.robot = robot
        self.group = group
        self.eef_link = eef_link

        self.bridge = CvBridge()
        self.start = rospy.ServiceProxy('o2as_phoxi_camera/start_acquisition', Trigger)
        self.trigger = rospy.ServiceProxy('o2as_phoxi_camera/trigger_frame', Trigger)
        self.get = rospy.ServiceProxy('o2as_phoxi_camera/get_frame', SetInt)
        self.stop = rospy.ServiceProxy('o2as_phoxi_camera/stop_acquisition', Trigger)

        self.depth_image = None

        ################################################################################
        # Publish and Subscribe topics
        ################################################################################
        rospy.Subscriber('o2as_phoxi_camera/depth_map', Image, self.depth_image_callback)

        ################################################################################
        # Register service callback
        ################################################################################
        self.get_image_service = rospy.Service('aist_kitting/get_image', GetImage, self.get_image_callback)
        self.search_service = rospy.Service('aist_kitting/search', Search, self.search_callback)
        self.pick_service = rospy.Service('aist_kitting/pick', Pick, self.pick_callback)
        self.move_named_pose_service = rospy.Service('aist_kitting/move_named_pose', MoveNamedPose, self.move_named_pose_callback)

        rospy.loginfo('AistKitting started up')
        rospy.spin()
    

    ################################################################################
    # methods to move robot
    ################################################################################


    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats, a Pose or a PoseStamped
        @param: actual     A list of floats, a Pose or a PoseStamped
        @param: tolerance  A float
        @returns: bool
        """

        all_equal = True
        if type(goal) is list:
            for index in range(len(goal)):
                if abs(actual[index] - goal[index]) > tolerance:
                    return False
        elif type(goal) is geometry_msgs.msg.PoseStamped:
            return self.all_close(goal.pose, actual.pose, tolerance)
        elif type(goal) is geometry_msgs.msg.Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

        return True

    def go_to_pose_goal(self, pose_goal_stamped):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group
        group.set_pose_target(pose_goal_stamped)  # How to set multiple goals? Hm.

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        current_pose = self.group.get_current_pose().pose
        return self.all_close(pose_goal_stamped.pose, current_pose, 0.01)

    def go_to_named_goal(self, goal_name):
        group = self.group
        group.set_named_target(goal_name)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        # current_pose = self.group.get_current_pose().pose
        # return self.all_close(pose_goal_stamped.pose, current_pose, 0.01)

    ################################################################################
    # other
    ################################################################################
    def depth_image_callback(self, data):
        self.depth_img = self.bridge.imgmsg_to_cv2(data, '32FC1')

    ################################################################################
    # Service callback
    ################################################################################

    def get_image_callback(self, req):
        rospy.loginfo('get_image started.')
 
        res_start = self.start()
        rospy.sleep(1)
       
        res_trigger = self.trigger()
        rospy.sleep(3)
        rospy.loginfo("triggered")
        res_get = self.get(0)
        rospy.sleep(1)
        if not (res_trigger.success and res_get.success):
            rospy.sleep(5)
            self.trigger()
            self.get(0)

        pcloud_filename = rospy.get_param('o2as_phoxi_camera/id')[1:-1]+".tiff"
        io.imsave(os.environ.get('ROS_HOME')+"/data/"+pcloud_filename, self.depth_img)

        res_stop = self.stop()
        rospy.loginfo('get_image finished.')

        return GetImageResponse(True, '1711015.tiff')
        

    def search_callback(self, req):
        rospy.loginfo("search started.")

        fge = rospy.ServiceProxy('/FGE', Search)
        res_fge = fge(req)
        rospy.loginfo("search finished.")
        res_fge = SearchResponse()
        return res_fge

    def pick_callback(self, req):
        rospy.loginfo("pick started.")

        robot_name = req.robot_name
        ee_link_name = req.ee_link_name
        frame_id = req.frame_id

        self.group = moveit_commander.MoveGroupCommander(robot_name)
        self.group.set_end_effector_link(robot_name + '_' + ee_link_name)
        self.group.set_planning_time(0.5)
        self.group.set_planner_id("RRTConnectkConfigDefault")
        
        # if self.go_to_named_goal("home_a"):
        #     rospy.sleep(3)
        
        # The robot approaches to object from just above.
        pose_goal = copy.deepcopy(req.goal)
        pose_goal.pose.orientation.x = -0.5
        pose_goal.pose.orientation.y = 0.5
        pose_goal.pose.orientation.z = 0.5
        pose_goal.pose.orientation.w = 0.5

        pose_goal.pose.position.z += 0.1
        pose_goal.header.frame_id = frame_id
        if self.go_to_pose_goal(pose_goal):
            rospy.sleep(3)
        
        pose_goal.pose.position.z -= 0.1
        pose_goal.header.frame_id = frame_id
        if self.go_to_pose_goal(pose_goal):
            rospy.sleep(3)

        pose_goal.pose.position.z += 0.1
        pose_goal.header.frame_id = frame_id
        if self.go_to_pose_goal(pose_goal):
            rospy.sleep(3)

        rospy.loginfo("pick finished.")
        return True

    def move_named_pose_callback(self, req):
        rospy.loginfo("move_named_pose started.")

        robot_name = req.robot_name
        ee_link_name = req.ee_link_name
        named_pose = req.named_pose

        group = moveit_commander.MoveGroupCommander(robot_name)
        group.set_end_effector_link(robot_name + ee_link_name)
        group.set_planning_time(0.5)
        group.set_planner_id("RRTConnectkConfigDefault")
        
        self.go_to_named_goal(named_pose)

        return True


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('aist_kitting')
    try:
        ne = AistKitting()
    except rospy.ROSInterruptException: pass
