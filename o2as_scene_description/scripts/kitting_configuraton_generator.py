#!/usr/bin/env python

"""Make kitting configuration files from order file.

    Inputs:
        order_file_name Kitting parts list which is offered from the committee.

    Outputs:
        part_bin_list.yaml This file offers ros parameter about the pair of parts_name and bin_name.
        placement_bin_layout.csv This file offers how to layout all parts bin in the rack.
"""

import os
import csv
import yaml
from operator import attrgetter

import rospkg
import rospy

rp = rospkg.RosPack()

class PartsInfo(object):
    def __init__(self, parts_name, bin_type):
        self.parts_name = parts_name
        self.bin_type = bin_type
        self.bin_name = ""

    def __repr__(self):
        return repr((self.parts_name, self.bin_type, self.bin_name))

bin_type = {
    4 : "bin_2",
    5 : "bin_2",
    6 : "bin_3",
    7 : "bin_2",
    8 : "bin_2",
    9 : "bin_1",
    10: "bin_1",
    11: "bin_2",
    12: "bin_1",
    13: "bin_2",
    14: "bin_1",
    15: "bin_1",
    16: "bin_1",
    17: "bin_1",
    18: "bin_1"
}

def write_part_bin_list(parts_bin_dict):
    """write part_bin_list to yaml.
    """

    with open(os.path.join(rp.get_path("o2as_routines"), "config", "kitting_parts_bin_list.yaml"), 'w') as f:
        yaml.dump(parts_bin_dict, f)

def convert_parts_info_to_dict(bin_layout):
    parts_info_dict = dict()
    parts_info_dict["part_bin_list"] = dict()

    for b in bin_layout:
        parts_info_dict["part_bin_list"][int(b.parts_name.strip("parts_"))] = b.bin_name

    return parts_info_dict

def write_bin_layout(bin_layout):
    """write bin layout in the kitting scene to placement_bin_layout.csv

        Returns:
            boolean value which is True if this function finished without errors.
    """

    header = ['parts_name', 'bin_type', 'bin_name']

    with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", "placement_bin_layout.csv"), "w") as f:
        writer = csv.writer(f)
        writer.writerow(header)
 
        for b in bin_layout:
            writer.writerow([b.parts_name, b.bin_type, b.bin_name])

    return True

def identify_bin(sorted_parts_info_list):
    bin_num = 1
    prev_bin_type = None

    for p in sorted_parts_info_list:
        if prev_bin_type != p.bin_type:
            bin_num = 1 
        else:
            bin_num += 1
        p.bin_name = p.bin_type + "_" + str(bin_num)
        prev_bin_type = p.bin_type

    return sorted_parts_info_list

def sort_parts_by_bin_type(parts_info_list, reverse=False):
    return sorted(parts_info_list, key=attrgetter('bin_type'), reverse=reverse)

def make_parts_info_list(part_set):
    parts_info_list = list()

    for part in part_set:
        parts_info_list.append(PartsInfo("parts_"+str(part), bin_type[part]))

    return parts_info_list

def read_number_of_part():
    """count number of parts from order file.

        Params:

        Returns:
        (parts_set) how many parts exists our order.

        Checked by Yuma Hijioka at 10/04/2018.
    """
    parts_set = set()

    with open(os.path.join(rp.get_path("o2as_scene_description"), "config", "kitting_order_file.csv"), 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        # [0, 1, 2, 3] = ["Set", "No.", "ID", "Name", "Note"]
        for data in reader:
            parts_set.add(int(data[2]))
    return parts_set

def main():
    parts_set = read_number_of_part()
    parts_info_list = make_parts_info_list(parts_set)

    parts_info_list = identify_bin(sort_parts_by_bin_type(parts_info_list, reverse=True))
    write_bin_layout(parts_info_list)

    parts_bin_dict = convert_parts_info_to_dict(parts_info_list)
    write_part_bin_list(parts_bin_dict)

if __name__ == '__main__':
    main()
