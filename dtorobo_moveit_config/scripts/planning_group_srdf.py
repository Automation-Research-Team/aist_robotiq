#!/usr/bin/env python

# TODO: Use urdf_parser_py.urdf instead. I gave it a try, but got
#  Exception: Required attribute not set in XML: upper
# upper is an optional attribute, so I don't understand what's going on
# See comments in https://github.com/ros/urdfdom/issues/36

import xml.dom.minidom
from math import pi

import rospy


def get_planning_groups(key='/torobo/robot_description_semantic', use_smallest_joint_limits=True):
    use_small = use_smallest_joint_limits
    use_mimic = True

    # Code inspired on the joint_state_publisher package by David Lu!!!
    # https://github.com/ros/robot_model/blob/indigo-devel/
    # joint_state_publisher/joint_state_publisher/joint_state_publisher
    description = rospy.get_param(key)

    return get_planning_groups_from_text(description)


def get_planning_groups_from_text(description):
    robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
    free_joints = {}
    dependent_joints = {}

    planning_groups = {}

    # Find all non-fixed joints
    for child in robot.childNodes:
        if child.nodeType is child.TEXT_NODE:
            continue
        if child.localName == 'group':
            name = child.getAttribute('name')
            attribute = {}

            attribute['is_chain'] = True

            if len(child.getElementsByTagName('group')) > 0:
                attribute['is_chain'] = False
            if ('arm' not in name) and ('torso_head' not in name):
                attribute['is_chain'] = False

            planning_groups[name] = attribute

    return planning_groups
