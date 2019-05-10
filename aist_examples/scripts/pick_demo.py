#!/usr/bin/env python
import copy
from math import pi, radians

import rospy
import actionlib
import tf
import robotiq_msgs.msg
import geometry_msgs.msg

import aist_graspability.msg
import aist_skills.msg


if __name__ == "__main__":
    rospy.init_node('pick_demo', anonymous=True)

    fge_action_client = actionlib.SimpleActionClient('aist_graspability/search_grasp_from_phoxi', aist_graspability.msg.SearchGraspFromPhoxiAction)
    moveLin_action_client = actionlib.SimpleActionClient('aist_skills/move_lin', aist_skills.msg.MoveLinAction)
    listener = tf.TransformListener()
    robotiq_action_client = actionlib.SimpleActionClient('a_bot_gripper/gripper_action_controller', robotiq_msgs.msg.CModelCommandAction)

    scene_path = '/home/mrg/ur-o2as/catkin_ws/src/aist_graspability/aist_graspability/data/sample.tif'
    mask_path = '/home/mrg/ur-o2as/catkin_ws/src/aist_graspability/aist_graspability/data/imr3.png'
    part_id = 13
    bin_name = 'o2as_ground'
    gripper_type = 'two_finger'
    algorithm = 'fge'
    take_new_image = True

    rospy.loginfo('search grasp candidates')
    goal_fge = aist_graspability.msg.SearchGraspFromPhoxiGoal()
    goal_fge.scene_path = scene_path
    goal_fge.mask_path = mask_path
    goal_fge.part_id = part_id
    goal_fge.bin_name = bin_name
    goal_fge.gripper_type = gripper_type
    goal_fge.algorithm = algorithm
    goal_fge.take_new_image = take_new_image
    fge_action_client.send_goal_and_wait(goal_fge)
    res_fge = fge_action_client.get_result()

    target_point = geometry_msgs.msg.PointStamped()
    target_point.header.frame_id = 'a_phoxi_m_sensor'
    target_point.point = res_fge.pos3D[0]
    target_rotipz = res_fge.rotipz[0]
    target_point_world = listener.transformPoint('o2as_ground', target_point)

    rospy.loginfo('above the target')
    goal_moveLin = aist_skills.msg.MoveLinGoal()
    goal_moveLin.group_name = 'a_bot'
    goal_moveLin.frame_id = 'o2as_ground'
    goal_moveLin.position = copy.deepcopy(target_point_world)
    goal_moveLin.position.z += 0.03
    goal_moveLin.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(radians(-90), radians(90), radians(0)))
    goal_moveLin.speed = 1.0
    moveLin_action_client.send_goal_and_wait(goal_moveLin)
    res_moveLin = moveLin_action_client.get_result()

    rospy.loginfo('approach to target')
    goal_moveLin = aist_skills.msg.MoveLinGoal()
    goal_moveLin.group_name = 'a_bot'
    goal_moveLin.frame_id = 'o2as_ground'
    goal_moveLin.position = copy.deepcopy(target_point_world)
    goal_moveLin.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.euler_from_quaternion(radians(target_rotipz - 90), radians(90), radians(0)))
    goal_moveLin.speed = 0.1
    moveLin_action_client.send_goal_and_wait(goal_moveLin)
    res_moveLin = moveLin_action_client.get_result()

    rospy.loginfo('grasp the target')
    goal_gripper = robotiq_msgs.msg.CModelCommandGoal()
    goal_gripper.position = 0.0
    goal_gripper.velocity = 1.0
    goal_gripper.force = 100.0
    robotiq_action_client.send_goal_and_wait(goal_gripper)
    res_gripper = robotiq_action_client.get_result()

    rospy.loginfo('above the target')
    goal_moveLin = aist_skills.msg.MoveLinGoal()
    goal_moveLin.group_name = 'a_bot'
    goal_moveLin.frame_id = 'o2as_ground'
    goal_moveLin.position = copy.deepcopy(target_point_world)
    goal_moveLin.position.z += 0.03
    goal_moveLin.orientation = geometry_msgs.msg.Quaternion(*tf.transformations.quaternion_from_euler(radians(-90), radians(90), radians(0)))
    goal_moveLin.speed = 1.0
    moveLin_action_client.send_goal_and_wait(goal_moveLin)
    res_moveLin = moveLin_action_client.get_result()

    print('pick finished!')
