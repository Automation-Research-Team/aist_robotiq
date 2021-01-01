#!/usr/bin/env python

import rospy
import os
import sys
import yaml
import argparse
import copy
from planning_group_srdf import get_planning_groups_from_text


def main(args):
    if os.path.exists(args.dump_file):
        os.remove(args.dump_file)

    f = open(os.path.join(args.robot_config) , 'r')
    robot_config = yaml.load(f)
    f.close()

    f = open(os.path.join(args.model_dir, robot_config['model'], args.srdf))
    srdf_text = f.read()
    f.close()

    groups = get_planning_groups_from_text(srdf_text)

    f = open(args.kinematics_yaml, 'r')
    kinematics = yaml.load(f)
    f.close()

    output = {}
    for group, attributes in groups.items():
        if attributes['is_chain']:
            output[group] = kinematics

    f = open(args.dump_file, "w")
    f.write(yaml.dump(output, default_flow_style=False))
    f.close()


def parse_arguments(args):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_config", type=str, default="")
    parser.add_argument("--srdf", type=str, default="")
    parser.add_argument("--model_dir", type=str, default="")
    parser.add_argument("--kinematics_yaml", type=str, default="")
    parser.add_argument("--dump_file", type=str, default="/tmp/tmp.yaml")
    return parser.parse_args(args)


if __name__ == '__main__':
    args = parse_arguments(rospy.myargv()[1:])
    main(args)
