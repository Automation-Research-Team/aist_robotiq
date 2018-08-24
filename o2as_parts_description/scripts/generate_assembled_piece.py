#!/usr/bin/python
import csv
import os

import rospy
import rospkg
rp = rospkg.RosPack()

# Read in the assembly matings
mating_filename = os.path.join(rp.get_path("o2as_parts_description"), "urdf/templates", "frames_to_mate.csv")
frame_matings = []
with open(mating_filename, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        if row:     # Should skip empty rows
            if row[0][0] == "#":    # Skips commented rows
                continue
            row_stripped = []
            for el in row:
                row_stripped.append(el.strip())       # Removes whitespaces
            frame_matings.append(row_stripped)

template_filename = os.path.join(rp.get_path("o2as_parts_description"), "urdf/templates", "assembly_template.urdf")
f = open(template_filename,'r')
template_front = f.read()
f.close()

# Write the file containing connections between the frames in the instructions
out_dir = os.path.join(rp.get_path("o2as_parts_description"), "urdf")
outfile = open(os.path.join(out_dir, "full_assembly.urdf"),'w+')
content = template_front

content +=  "    <xacro:assy_part_01 prefix=\"${prefix}\" parent=\"${parent}\" spawn_attached=\"true\"> \n"
content +=  "      <xacro:insert_block name=\"origin\"/> \n"
content +=  "    </xacro:assy_part_01> \n"
content +=  "    \n"
for mating in frame_matings:
    print(mating)
    parent_frame = mating[0]
    child_frame = mating[1]
    parent_part_num = int(parent_frame[10:12])
    child_part_num = int(child_frame[10:12])

    # First, spawn the part unattached (no link to the world). Then, create the mating joint manually.
    new_mating = "" 
    new_mating +=  "    <xacro:assy_part_" + str(child_part_num).zfill(2)+ " prefix=\"${prefix}\" parent=\"${prefix}" + parent_frame + "\" spawn_attached=\"false\"> \n"
    new_mating +=  "      <origin rpy=\"" + mating[2] + " " + mating[3] + " " + mating[4] + "\" xyz=\"" + mating[5] + " " + mating[6] + " " + mating[7] + " \"/> \n"
    new_mating +=  "    </xacro:assy_part_" + str(child_part_num).zfill(2)+ "> \n"
    new_mating +=  "    \n"
    new_mating +=  "    <joint name=\"${prefix}part_" + str(parent_part_num).zfill(2) + "_to_" + str(child_part_num).zfill(2) + "_joint\" type=\"fixed\"> \n"
    new_mating +=  "      <parent link=\"${prefix}" + parent_frame + "\"/> \n"
    new_mating +=  "      <child link=\"${prefix}" + child_frame + "\"/> \n"
    new_mating +=  "      <origin rpy=\"" + mating[2] + " " + mating[3] + " " + mating[4] + "\" xyz=\"" + mating[5] + " " + mating[6] + " " + mating[7] + " \"/> \n"
    new_mating +=  "    </joint> \n"
    new_mating +=  "    \n"
    content += new_mating

content += "  </xacro:macro>\n"
content += "</robot>\n"
outfile.write(content)





