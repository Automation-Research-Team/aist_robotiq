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

import rospkg
import rospy

rp = rospkg.RosPack()

bin_type = {
    4 : bin_2,
    5 : bin_2,
    6 : bin_3,
    7 : bin_2,
    8 : bin_2,
    9 : bin_1,
    10: bin_1,
    11: bin_2,
    12: bin_1,
    13: bin_2,
    14: bin_1,
    15: bin_1,
    16: bin_1,
    17: bin_1,
    18: bin_1
}

def write_bin_layout():
    """write bin layout in the kitting scene to placement_bin_layout.csv

        Returns:
            boolean value which is True if this function finished without errors.
    """

    header = ['parts_name', 'set', 'bin_type' , 'bin_name']

    with open(os.path.join(rp.get_path("o2as_scene_description"), "config", "placement_bin_layout_auto.csv"), "w") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        for b in bin_set:
            writer.writerow([b.parts_name, b.bin_set, b.bin_type, b.bin_name])


    return True

def read_number_of_part():
    """count number of parts from order file.

        Params:

        Returns:
        (parts_set) how many parts exists our order.
    """
    part_set = set()

    with open(os.path.join(rp.get_path("o2as_scene_description"), "config", "kitting_order_file.csv"), 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        # [0, 1, 2, 3] = ["Set", "No.", "ID", "Name", "Note"]
        for data in reader:
            part_set.add("part_" + data[2])

def main():
    write_bin_layout()

if __name__ == '__main__':
    main()
