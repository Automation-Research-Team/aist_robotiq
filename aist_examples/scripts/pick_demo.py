#!/usr/bin/env python
import os
import copy
from math import pi, radians
from datetime import datetime as dt

import rospy
import actionlib
import tf
import robotiq_msgs.msg
import geometry_msgs.msg
import rospkg

import aist_graspability.msg
import aist_skills.msg

rp = rospkg.RosPack()

def show_current_pose(feedback):
    rospy.loginfo('current pose')
    rospy.loginfo(feedback)

def attempt(scene_path, mask_path, part_id, gripper_type, algorithm):
    # Search grasp candidates
    goal_fge = aist_graspability.msg.SearchGraspFromPhoxiGoal()
    goal_fge.scene_path = scene_path
    goal_fge.mask_path = mask_path
    goal_fge.part_id = part_id
    goal_fge.bin_name = 'o2as_ground'
    goal_fge.gripper_type = gripper_type
    goal_fge.algorithm = algorithm
    goal_fge.take_new_image = True
    vision_action_client.send_goal_and_wait(goal_fge)

    res_fge = vision_action_client.get_result()

    # Pick
    goal_pick = aist_skills.msg.PickGoal()
    goal_pick.group_name = 'a_bot'
    goal_pick.frame_id = 'a_phoxi_m_sensor'
    goal_pick.position = copy.deepcopy(res_fge.pos3D[0])
    goal_pick.orientation = geometry_msgs.msg.Point(res_fge.rotiqz[0], 90, 0)
    goal_pick.approach_offset = 0.03
    goal_pick.grasp_offset = 0.003
    goal_pick.speed_fast = 1.0
    goal_pick.speed_slow = 0.1
    pick_action_client.send_goal_and_wait(goal_pick, feedback_cb=show_current_pose)
    res_pick = pick_action_client.get_result()

    # Place
    goal_place = aist_skills.msg.PlaceGoal()
    goal_place.group_name = 'a_bot'
    goal_place.frame_id = 'o2as_ground'
    goal_place.position = geometry_msgs.msg.Point(-0.35, 0.45, 0.10)
    goal_place.orientation = geometry_msgs.msg.Point(res_fge.rotipz[0], 90, 0)
    goal_place.approach_height = 0.15
    goal_place.release_height = 0.05
    goal_place.speed_fast = 1.0
    goal_place.speed_slow = 0.1
    place_action_client.send_goal_and_wait(goal_place, feedback_cb=show_current_pose)

def init():
    rospy.init_node('pick_demo', anonymous=False)

    # Action clients
    vision_action_client = actionlib.SimpleActionClient('aist_graspability/search_grasp_from_phoxi',
                                                            aist_graspability.msg.SearchGraspFromPhoxiAction)
    vision_action_client.wait_for_server()
    pick_action_client = actionlib.SimpleActionClient('aist_skills/pick',
                                                      aist_skills.msg.PickAction)
    pick_action_client.wait_for_server()
    place_action_client = actionlib.SimpleActionClient('aist_skills/place',
                                                    aist_skills.msg.PlaceAction)
    place_action_client.wait_for_server()


if __name__ == "__main__":
    mask_path = os.path.join(rp.get_path('aist_graspability'), 'data/imr3.png')

    tdatetime = dt.now()
    tstr = tdatetime.strftime('%Y%m%d-%H%M%S')
    init()

    for i in xrange(num_attempt):
        scene_file = os.path.join(rospack.get_path('aist_graspability'), 'data', tstr + '_' + i + '.tif')
        attempt(scene_file, mask_path, part_id, gripper_type, algorithm)
