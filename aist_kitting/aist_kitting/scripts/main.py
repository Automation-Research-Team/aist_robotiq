#!/usr/bin/env python

import numpy as np

import rospy
import tf

# from aist_kitting_msgs.srv import *
from aist_kitting_msgs.srv import *
from geometry_msgs.msg import PoseStamped

def pick_test():
    pick = rospy.ServiceProxy("aist_kitting/pick", Pick)
    req_pick = PickRequest()
    req_pick.robot_name = "a_bot"
    req_pick.ee_link_name = "dual_suction_gripper_pad_link"
    req_pick.frame_id = "world"
    req_pick.goal = PoseStamped()
    req_pick.goal.pose.position.x = 0.0
    req_pick.goal.pose.position.y = 0.0
    # req_pick.goal.pose.position.z = 0.825
    req_pick.goal.pose.position.z = 1.0

    res_pick = pick(req_pick)
    rospy.loginfo(res_pick)

    req_pick.goal.pose.position.x = 0.3
    res_pick = pick(req_pick)
    rospy.loginfo(res_pick)

    req_pick.goal.pose.position.y = 0.3
    rospy.loginfo(req_pick.goal.pose)
    res_pick = pick(req_pick)
    rospy.loginfo(res_pick)

    # req_pick.goal.pose.position.x = -0.3
    # res_pick = pick(req_pick)
    # rospy.loginfo(res_pick)

    # req_pick.goal.pose.position.y = 0.0
    # res_pick = pick(req_pick)
    # rospy.loginfo(res_pick)

def move_named_pose_test():
    rospy.loginfo("move_named_pose_test")
    move_named_pose = rospy.ServiceProxy("aist_kitting/move_named_pose", MoveNamedPose)

    req_move_named_pose = MoveNamedPoseRequest()
    req_move_named_pose.robot_name = "a_bot"
    req_move_named_pose.ee_link_name = "dual_suction_gripper_pad_link"
    req_move_named_pose.named_pose = "home_a"

    res_move_named_pose = move_named_pose(req_move_named_pose)
    rospy.sleep(1)
    rospy.loginfo(res_move_named_pose)


def search_test():
    get_image = rospy.ServiceProxy("aist_kitting/get_image", GetImage)
    # search = rospy.ServiceProxy("aist_kitting/search", Search)rospy.ServiceProxy("aist_kitting/move_named_pose", MoveNamedPose)

    res_get_image = get_image()
    req_search = SearchRequest()
    req_search.part_id = 5
    req_search.pcloud_filename = res_get_image.pcloud_filename
    res_search = search(req_search)
    rospy.loginfo(res_search)

if __name__ == '__main__':
    rospy.init_node("aist_kitting_demo")

    pick_test()
    # search_test()
    move_named_pose_test()

    # T_phoxi = np.matrix([
    #     [-0.005532755388, 0.895263942347, -0.445501809369, 544.512175068866],
    #     [0.986069642027, -0.069178481132, -0.151264664816, 157.894106209722],
    #     [-0.166240938674, -0.440132720076, -0.882409847535, 1465.092961368773],
    #     [0, 0, 0, 1]
    # ])
    # q = tf.transformations.quaternion_from_matrix(T_phoxi)
