#!/usr/bin/env python

import numpy as np

import rospy
import tf
import tf_conversions
from math import *

from aist_kitting_msgs.srv import *
# from o2as_usb_relay.srv import SetPower
from o2as_usb_relay.srv import *

from geometry_msgs.msg import PoseStamped

move_to_goal_pose = rospy.ServiceProxy("aist_kitting/move_to_goal_pose", MoveToGoalPose)

def move_to_goal_pose_test(req):

    move_to_goal_pose = rospy.ServiceProxy("aist_kitting/move_to_goal_pose", MoveToGoalPose)
    res_move_to_goal_pose = move_to_goal_pose(req)

    rospy.loginfo(res_move_to_goal_pose)

def pick_test():
    pick = rospy.ServiceProxy("aist_kitting/pick", Pick)
    req_pick = PickRequest()
    req_pick.robot_name = "b_bot"
    req_pick.ee_link_name = "dual_suction_gripper_pad_link"
    req_pick.frame_id = "world"
    req_pick.goal = PoseStamped()
    req_pick.goal.pose.position.x = 0.0
    req_pick.goal.pose.position.y = 0.0
    # req_pick.goal.pose.position.z = 0.825
    req_pick.goal.pose.position.z = 0.7

    res_pick = pick(req_pick)
    rospy.loginfo(res_pick)

def move_named_pose_test():
    rospy.loginfo("move_named_pose_test")
    move_named_pose = rospy.ServiceProxy("aist_kitting/move_named_pose", MoveNamedPose)

    req_move_named_pose = MoveNamedPoseRequest()
    req_move_named_pose.robot_name = "b_bot"
    req_move_named_pose.ee_link_name = "dual_suction_gripper_pad_link"
    req_move_named_pose.named_pose = "home"

    res_move_named_pose = move_named_pose(req_move_named_pose)
    rospy.sleep(1)
    rospy.loginfo(res_move_named_pose)

def search_test():
    get_image = rospy.ServiceProxy("aist_kitting/get_image", GetImage)
    search = rospy.ServiceProxy("aist_kitting/search", Search)

    res_get_image = get_image()
    req_search = SearchRequest()
    req_search.part_id = 5
    req_search.pcloud_filename = res_get_image.pcloud_filename
    res_search = search(req_search)
    rospy.loginfo(res_search)

    return res_search

def suction_test():
    rospy.wait_for_service("o2as_usb_relay_server/set_power")
    suction_trigger = rospy.ServiceProxy("o2as_usb_relay_server/set_power", SetPower)
    
    req_suction_trigger = SetPowerRequest()
    req_suction_trigger.port = 1
    req_suction_trigger.on = True

    res_suction_trigger = suction_trigger(req_suction_trigger)
    rospy.loginfo(res_suction_trigger.message)
    if res_suction_trigger.success:
        rospy.sleep(3)

    req_suction_trigger.on = False
    res_suction_trigger = suction_trigger(req_suction_trigger)
    rospy.loginfo(res_suction_trigger.message)
    if res_suction_trigger.success:
        rospy.sleep(3)

def phoxi_transformation_examples():
    # original matrix
    T_phoxi = np.array([
        [-0.045899001285, 0.902472945015, -0.428294133975, 0.594556671213511],
        [0.984380843543, -0.032086403336, -0.173103488082, 0.139281989152648 ],
        [-0.169963633010, -0.429549818109, -0.886904345020, 1.467133761064637 ],
        [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000 ]
    ])

    # https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py
    # Example code for getting rpy angles and translation from the 4x4 homogenous matrix via TF
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(T_phoxi)
    print(angles)
    print(trans)

    rpy = tf.transformations.euler_from_matrix(T_phoxi)
    xyz = [T_phoxi[0,3], T_phoxi[1,3], T_phoxi[2,3]]
    print(rpy)
    print(xyz)

    # T_phoxi is the transformation to the camera frame
    # The transformation to be entered into the URDF is to the camera base frame

    # One possibility: Take the transformation between camera frame and camera base_frame from the URDF,
    # and add it to the transformation here manually.

    rpy_camera_frame_to_base = [pi/2, pi/2, pi/2]
    xyz_camera_frame_to_base = [1, 2, 3]
    T_camera_frame_to_base = tf.transformations.compose_matrix(scale=None, shear=None, angles=rpy_camera_frame_to_base, 
                        translate=xyz_camera_frame_to_base, perspective=None)

    T_phoxi_base = tf.transformations.concatenate_matrices(T_phoxi, T_camera_frame_to_base)
    # Now rpy and xyz can be extracted for the T_phoxi_base transformation and used in the scene URDF,
    # as the transformation from world to camera base.
    # NOTE: The signs and multiplications in the code above may have to be inverted.

    return

def transform_phoxi_to_world(pose):
    # original matrix
    # T_phoxi = np.array([
    #     [-0.045899001285, 0.902472945015, -0.428294133975, 0.600556671213511],
    #     [0.984380843543, -0.032086403336, -0.173103488082, 0.139281989152648 ],
    #     [-0.169963633010, -0.429549818109, -0.886904345020, 1.467133761064637 ],
    #     [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000 ]
    # ])
    T_phoxi = np.array([
        [-0.045899001285, 0.902472945015, -0.428294133975, 0.594556671213511],
        [0.984380843543, -0.032086403336, -0.173103488082, 0.139281989152648 ],
        [-0.169963633010, -0.429549818109, -0.886904345020, 1.467133761064637 ],
        [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000 ]
    ])

    position = np.dot(T_phoxi, np.array([[pose.position.x],[pose.position.y],[pose.position.z],[1]]))
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]

    # pose_orient_mat = tf.transformations.quaternion_matrix(pose.orientation)
    # orientation = tf.transformations.quaternion_from_matrix(np.dot(T_phoxi, pose_orient_mat))
    # pose.orientation[0] = orientation[0]
    # pose.orientation[1] = orientation[1]
    # pose.orientation[2] = orientation[2]
    # pose.orientation[3] = orientation[3]
    rospy.loginfo(pose)

    return pose

def move_midterm_position():
    goal = MoveToGoalPoseRequest()
    goal.robot_name = "b_bot"
    goal.ee_link_name = "dual_suction_gripper_pad_link"
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.x = -0.5
    goal_pose.pose.orientation.y = 0.5
    goal_pose.pose.orientation.z = 0.5
    goal_pose.pose.orientation.w = 0.5
    goal_pose.pose.position.x = 0.2
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 1.0
    goal_pose.header.frame_id = "world"
    goal.goal = goal_pose

    res_move_to_goal_pose = move_to_goal_pose(goal)
    rospy.loginfo(res_move_to_goal_pose)

def main():
    

    # Go to initial pose.
    move_named_pose = rospy.ServiceProxy("aist_kitting/move_named_pose", MoveNamedPose)
    req_move_named_pose = MoveNamedPoseRequest()
    req_move_named_pose.robot_name = "b_bot"
    req_move_named_pose.ee_link_name = "dual_suction_gripper_pad_link"
    req_move_named_pose.named_pose = "home"
    res_move_named_pose = move_named_pose(req_move_named_pose)
    rospy.sleep(5)
    rospy.loginfo(res_move_named_pose)

    # pick_test()

    # Search object.
    get_image = rospy.ServiceProxy("aist_kitting/get_image", GetImage)
    search = rospy.ServiceProxy("aist_kitting/search", Search)
    res_get_image = get_image()
    req_search = SearchRequest()
    req_search.part_id = 5
    req_search.pcloud_filename = res_get_image.pcloud_filename
    res_search = search(req_search)
    rospy.loginfo("search result: ")
    rospy.loginfo(res_search)

    goal = PoseStamped()
    goal.pose.position = res_search.pos3D[0]
    rospy.loginfo("goal: " + str(goal))
    goal.pose.orientation = tf.transformations.quaternion_from_euler(res_search.rot3D[0].x, res_search.rot3D[0].y, res_search.rot3D[0].z)
    rospy.loginfo(goal)
    
    # Convert target pose coordinates from camera to world.
    goal.pose = transform_phoxi_to_world(goal.pose)
    goal.header.frame_id = "world"
    rospy.loginfo(goal)

    # move midterm pose
    move_midterm_position()

    # Pick object.
    pick = rospy.ServiceProxy("aist_kitting/pick", Pick)
    req_pick = PickRequest()
    req_pick.robot_name = "b_bot"
    req_pick.ee_link_name = "dual_suction_gripper_pad_link"
    req_pick.frame_id = "world"
    req_pick.goal.pose.position = goal.pose.position
    # req_pick.goal.pose.orientation = goal.pose.orientation
    # req_pick.goal.pose.position.x = 0.0
    # req_pick.goal.pose.position.y = 0.0
    # req_pick.goal.pose.position.z = 0.825
    # req_pick.goal.pose.position.z = 1.0
    res_pick = pick(req_pick)
    rospy.loginfo(res_pick)

    move_midterm_position()

    # # Go to initial pose.
    # move_named_pose = rospy.ServiceProxy("aist_kitting/move_named_pose", MoveNamedPose)
    # req_move_named_pose = MoveNamedPoseRequest()
    # req_move_named_pose.robot_name = "b_bot"
    # req_move_named_pose.ee_link_name = "dual_suction_gripper_pad_link"
    # req_move_named_pose.named_pose = "home"
    # res_move_named_pose = move_named_pose(req_move_named_pose)
    # rospy.sleep(5)
    # rospy.loginfo(res_move_named_pose)




if __name__ == '__main__':
    rospy.init_node("aist_kitting_demo")
    phoxi_transformation_examples
    # main()

    # pick_test()
    # move_named_pose_test()
    # suction_test()
    # res_search = search_test()
    # move_named_pose_test()

    # ## Test that the robot move to correct position.
    # # common settings
    # goal = MoveToGoalPoseRequest()
    # goal.robot_name = "b_bot"
    # goal.ee_link_name = "dual_suction_gripper_pad_link"
    # goal_pose = PoseStamped()
    # goal_pose.pose.orientation.x = -0.5
    # goal_pose.pose.orientation.y = 0.5
    # goal_pose.pose.orientation.z = 0.5
    # goal_pose.pose.orientation.w = 0.5
    # # test 01
    # goal_pose.pose.position.x = 0.0
    # goal_pose.pose.position.y = 0.0
    # goal_pose.pose.position.z = 1.0
    # goal_pose.header.frame_id = "world"
    # goal.goal = goal_pose
    # move_to_goal_pose_test(goal)

    # # # test 02
    # # goal_pose.pose.position.x = 0.0
    # # goal_pose.pose.position.y = 0.25
    # # goal_pose.pose.position.z = 1.0
    # # goal.goal = goal_pose
    # # move_to_goal_pose_test(goal)

    # # # test 03
    # # goal_pose.pose.position.x = 0.0
    # # goal_pose.pose.position.y = -0.25
    # # goal_pose.pose.position.z = 1.0
    # # goal.goal = goal_pose
    # # move_to_goal_pose_test(goal)

