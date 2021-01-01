#!/usr/bin/env python

import rospy
import os
import sys
import yaml
import argparse
from joint_limits_urdf import get_joint_limits_from_text

import subprocess
import shlex

def main(args):
    if os.path.exists(args.dump_file):
        os.remove(args.dump_file)

    f = open(os.path.join(args.robot_config) , 'r')
    robot_config = yaml.load(f)
    f.close()

    f = open(os.path.join(args.model_dir, robot_config['model'], args.yaml), 'r')
    controller_list = yaml.load(f)
    f.close()

    cmd = args.xacro + " --inorder '" + args.urdf + "' path:=" + args.path

    p = subprocess.Popen(shlex.split(cmd), stdin=subprocess.PIPE, stdout=subprocess.PIPE)

    stdout_data = p.communicate()[0]

    limits = get_joint_limits_from_text(stdout_data)

    attribute = {}
    for key, value in controller_list.items():
        if key == 'joint_state_controller':
            continue

        for key2, value2 in value.items():
            if key2 == 'joints':
                for joint in value2:
                    attribute[joint] = {'has_acceleration_limits': False, 'has_velocity_limits': True, 'max_acceleration': 0, 'max_velocity': limits[joint]['max_velocity']}
            if key2 == 'joint':
                attribute[value2] = {'has_acceleration_limits': False, 'has_velocity_limits': True, 'max_acceleration': 0, 'max_velocity': limits[value2]['max_velocity']}

    output = {}
    output["joint_limits"] = attribute

    f = open(args.dump_file, "w")
    f.write(yaml.dump(output, default_flow_style=False))
    f.close()
    

def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_config", type=str, default="")
    parser.add_argument("--yaml", type=str, default="")
    parser.add_argument("--model_dir", type=str, default="")
    parser.add_argument("--xacro", type=str, default="")
    parser.add_argument("--urdf", type=str, default="")
    parser.add_argument("--path", type=str, default="")
    parser.add_argument("--dump_file", type=str, default="/tmp/tmp.yaml")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)
