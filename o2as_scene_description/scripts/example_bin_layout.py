#!/usr/bin/env python

import csv
import os

import rospy
import rospkg
rp = rospkg.RosPack()

def read_orderlist():

    part_set = set()
    with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", "ExampleOfSetListFile.csv"), 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        # [0, 1, 2, 3] = ["Set", "No.", "ID", "Name", "Note"]
        for data in reader:
            part_set.add("part_" + data[2])

    # print(part_set)

    part_bin_definition = dict()
    with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", "part_bin_definitions.csv"), 'r') as f:
        reader = csv.reader(f)
        header = next(reader)

        for data in reader:
            part_bin_definition[data[0]] = data[1]

    # print(part_bin_definition)

    part_bin_list = list()
    for part in part_set:
        part_bin_list.append([part, part_bin_definition[part]])
    
    # print(part_bin_list)

    # extract by part_bin_def


def main():
    os.chdir('../')
    directory = os.getcwd()

    read_orderlist()



if __name__ == '__main__':
    main()
