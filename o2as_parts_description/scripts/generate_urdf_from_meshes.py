#!/usr/bin/python

import csv
import os

import rospy
import rospkg
rp = rospkg.RosPack()

# Read in the files
filenames = os.listdir(os.path.join(rp.get_path("o2as_parts_description"), "meshes"))
filenames_strip1 = []
filenames_no_ext = []
for name in filenames:
    filenames_strip1.append(os.path.splitext(name)[0]) 
for name in filenames_strip1: 
    filenames_no_ext.append(os.path.splitext(name)[0])        # This removes the .vhacd from ".vhacd.dae" files
partnames = list(sorted(set(filenames_no_ext)))   # Removes duplicates and sorts (because sets do not allow duplicate entries)

# Read in the templates
extra_joint_filename = os.path.join(rp.get_path("o2as_parts_description"), "urdf/templates", "extra_frames.csv")
macro_template_filename = os.path.join(rp.get_path("o2as_parts_description"), "urdf/templates", "macro_template.urdf")
spawn_template_filename = os.path.join(rp.get_path("o2as_parts_description"), "urdf/templates", "spawn_template.urdf")
out_dir = os.path.join(rp.get_path("o2as_parts_description"), "urdf/generated")

f = open(macro_template_filename,'r')
macro_template = f.read()
f.close()
f = open(spawn_template_filename,'r')
spawn_template = f.read()
f.close()
extra_frames = []
with open(extra_joint_filename, 'r') as f:
  reader = csv.reader(f)
  header = next(reader)
  for row in reader:
    row_stripped = []
    for el in row:
      row_stripped.append(el.strip())       # Removes whitespaces
    extra_frames.append(row_stripped)

# Write the spawn files
# --- This is ignored for now

# Write the macros
for part_num, partname in enumerate(partnames):
    outfile = open(os.path.join(out_dir, partname+"_macro.urdf"),'w+')
    content = macro_template.replace("PARTNAME", partname)
    mname = "assy_part_" + partname[0:2]
    content = content.replace("MACRONAME", mname)     # This should be the number of the part

    if int(partname[0:2]) in [1,2,3]:
        content = content.replace("vhacd.dae", "stl")
        content = content.replace("dae", "stl")

    extra_frames_urdf = ""
    for entry in extra_frames:
        if int(entry[0]) == int(partname[0:2]):
            new_joint = ""
            new_joint +=  "    <joint name=\"${prefix}" + mname + "_LINK_NAME_joint\" type=\"fixed\"> \n"
            new_joint +=  "      <parent link=\"${prefix}" + mname + "\"/> \n"
            new_joint +=  "      <child link=\"${prefix}" + mname + "_LINK_NAME\"/> \n"
            new_joint +=  "      <origin rpy=\"${" + \
                                entry[2] + "} ${" + \
                                entry[3] + "} ${" + \
                                entry[4] + "}\" xyz=\"${" + \
                                entry[5] + "} ${" + \
                                entry[6] + "} ${" + \
                                entry[7] + "}\"/> \n"
            new_joint +=  "    </joint> \n"
            new_joint +=  "    <link name=\"${prefix}" + mname + "_LINK_NAME\"/> \n"
            new_joint +=  "    \n"
            new_joint = new_joint.replace("LINK_NAME", entry[1])
            extra_frames_urdf += new_joint
    if extra_frames_urdf:
        content = content.replace("<!-- #EXTRAFRAMES -->", extra_frames_urdf)
    outfile.write(content)
    rospy.loginfo("Wrote " + os.path.join(out_dir, partname+"_macro.urdf"))





