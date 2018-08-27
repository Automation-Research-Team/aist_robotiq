#!/usr/bin/python
import csv
import os

import rospy
import rospkg
import tf
import geometry_msgs
from math import pi
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

frames_filename = os.path.join(rp.get_path("o2as_parts_description"), "urdf/templates", "extra_frames.csv")
extra_frames = []
with open(frames_filename, 'r') as f:
    reader = csv.reader(f)
    header = next(reader)
    for row in reader:
        if row:     # Should skip empty rows
            if row[0][0] == "#":    # Skips commented rows
                continue
            row_stripped = []
            for el in row:
                row_stripped.append(el.strip())       # Removes whitespaces
            extra_frames.append(row_stripped)

template_filename = os.path.join(rp.get_path("o2as_parts_description"), "urdf/templates", "assembly_template.urdf")
f = open(template_filename,'r')
template_front = f.read()
f.close()

# Write the file containing connections between the frames in the instructions
out_dir = os.path.join(rp.get_path("o2as_parts_description"), "urdf/generated")
outfile = open(os.path.join(out_dir, "full_assembly.urdf"),'w+')
content = template_front

content +=  "    <xacro:assy_part_01 prefix=\"${prefix}\" parent=\"${parent}\" spawn_attached=\"true\"> \n"
content +=  "      <xacro:insert_block name=\"origin\"/> \n"
content +=  "    </xacro:assy_part_01> \n"
content +=  "    \n"
for mating in frame_matings:
    parent_frame = mating[0]
    child_frame = mating[1]
    parent_part_num = int(parent_frame[10:12])
    child_part_num = int(child_frame[10:12])
    child_part_base_frame = "assy_part_" + str(child_part_num).zfill(2)
    child_part_subframe_name = child_frame[13:]

    # Find the name of the child_frame in the extra_frames, and use its offset to link it to that part's base frame.
    # This is necessary because only one frame in the tree may be without a parent.
    for subframe_offset in extra_frames:
        if subframe_offset[1] == child_part_subframe_name and int(subframe_offset[0]) == child_part_num:
            subframe_offset = subframe_offset
            break
    
    # Get the inverse transform for the transformation entered into extra_frames
    rpy = [float(eval(subframe_offset[2])), float(eval(subframe_offset[3])), float(eval(subframe_offset[4]))]
    xyz = [float(eval(subframe_offset[5])), float(eval(subframe_offset[6])), float(eval(subframe_offset[7]))]
    q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    
    t = tf.TransformerROS(True, rospy.Duration(10.0))
    m = geometry_msgs.msg.TransformStamped()
    m.header.frame_id = child_frame             
    m.child_frame_id = child_part_base_frame   # I don't know why this order returns the correct results, it doesn't seem right.
    m.transform.translation.x = xyz[0]
    m.transform.translation.y = xyz[1]
    m.transform.translation.z = xyz[2]
    m.transform.rotation.x = q[0]
    m.transform.rotation.y = q[1]
    m.transform.rotation.z = q[2]
    m.transform.rotation.w = q[3]
    t.setTransform(m)

    mating_pose = geometry_msgs.msg.PoseStamped()
    mating_pose.header.frame_id = child_frame
    mating_pose.pose.position.x = float(eval(mating[5]))
    mating_pose.pose.position.y = float(eval(mating[6]))
    mating_pose.pose.position.z = float(eval(mating[7]))
    mating_pose.pose.orientation = geometry_msgs.msg.Quaternion(
                                    *tf.transformations.quaternion_from_euler(float(eval(mating[2])), float(eval(mating[3])), float(eval(mating[4]))) )
    mating_pose = t.transformPose(child_part_base_frame, mating_pose)
    
    rpy_in_base = tf.transformations.euler_from_quaternion(
                    [mating_pose.pose.orientation.x, mating_pose.pose.orientation.y, 
                     mating_pose.pose.orientation.z, mating_pose.pose.orientation.w] )

    # First, spawn the part unattached (no link to the world). Then, create the mating joint manually, offset to the part's base frame.
    new_mating = "" 
    new_mating +=  "    <xacro:assy_part_" + str(child_part_num).zfill(2)+ " prefix=\"${prefix}\" parent=\"${prefix}" + parent_frame + "\" spawn_attached=\"false\"> \n"
    new_mating +=  "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/> \n"
    new_mating +=  "    </xacro:assy_part_" + str(child_part_num).zfill(2)+ "> \n"
    new_mating +=  "    \n"
    new_mating +=  "    <joint name=\"${prefix}part_" + str(parent_part_num).zfill(2) + "_to_" + str(child_part_num).zfill(2) + "_joint\" type=\"fixed\"> \n"
    new_mating +=  "      <parent link=\"${prefix}" + parent_frame + "\"/> \n"
    new_mating +=  "      <child link=\"${prefix}" + child_part_base_frame + "\"/> \n"
    new_mating +=  "      <origin rpy=\"${" + \
                            str(rpy_in_base[0]) + "} ${" + \
                            str(rpy_in_base[1]) + "} ${" + \
                            str(rpy_in_base[2]) + "}\" xyz=\"${" + \
                            str(mating_pose.pose.position.x) + "} ${" + \
                            str(mating_pose.pose.position.y) + "} ${" + \
                            str(mating_pose.pose.position.z) + "}\"/> \n"
    new_mating +=  "    </joint> \n"
    new_mating +=  "    \n"
    content += new_mating

content += "  </xacro:macro>\n"
content += "</robot>\n"
outfile.write(content)





