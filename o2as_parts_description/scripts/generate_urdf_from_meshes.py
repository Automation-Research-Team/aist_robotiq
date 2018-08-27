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
macro_frames_only_template_filename = os.path.join(rp.get_path("o2as_parts_description"), "urdf/templates", "macro_frames_only_template.urdf")
spawn_template_filename = os.path.join(rp.get_path("o2as_parts_description"), "urdf/templates", "spawn_template.urdf")
out_dir = os.path.join(rp.get_path("o2as_parts_description"), "urdf/generated")

f = open(macro_template_filename,'r')
macro_template = f.read()
f.close()
f = open(macro_frames_only_template_filename,'r')
macro_frames_only_template = f.read()
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
    macrofile = open(os.path.join(out_dir, partname+"_macro.urdf"),'w+')
    macro_frames_only_file = open(os.path.join(out_dir, partname+"_frames_only_macro.urdf"),'w+')

    mname_int = "assy_part_" + partname[0:2]                    # = "assy_part_01" for part 01.
    mname_ext = mname_int
    mname_fo_int = mname_int
    mname_fo_ext = "assy_part_frames_only_" + partname[0:2]     # This only changes the name of the macro, not the joint/link names
    macrofile_content = macro_template.replace("PARTNAME", partname)
    macrofile_content = macrofile_content.replace("MACRONAME_INTERNAL", mname_int)
    macrofile_content = macrofile_content.replace("MACRONAME_EXTERNAL", mname_ext)
    macro_frames_only_filecontent = macro_frames_only_template.replace("MACRONAME_INTERNAL", mname_fo_int)
    macro_frames_only_filecontent = macro_frames_only_filecontent.replace("MACRONAME_EXTERNAL", mname_fo_ext)

    if int(partname[0:2]) in [1,2,3]:
        macrofile_content = macrofile_content.replace("vhacd.dae", "stl")
        macrofile_content = macrofile_content.replace("dae", "stl")

    extra_frames_urdf = ""
    for entry in extra_frames:
        if int(entry[0]) == int(partname[0:2]):
            new_joint = ""
            new_joint +=  "    <joint name=\"${prefix}" + mname_int + "_LINK_NAME_joint\" type=\"fixed\"> \n"
            new_joint +=  "      <parent link=\"${prefix}" + mname_int + "\"/> \n"
            new_joint +=  "      <child link=\"${prefix}" + mname_int + "_LINK_NAME\"/> \n"
            new_joint +=  "      <origin rpy=\"${" + \
                                entry[2] + "} ${" + \
                                entry[3] + "} ${" + \
                                entry[4] + "}\" xyz=\"${" + \
                                entry[5] + "} ${" + \
                                entry[6] + "} ${" + \
                                entry[7] + "}\"/> \n"
            new_joint +=  "    </joint> \n"
            new_joint +=  "    <link name=\"${prefix}" + mname_int + "_LINK_NAME\"/> \n"
            new_joint +=  "    \n"
            new_joint = new_joint.replace("LINK_NAME", entry[1])
            extra_frames_urdf += new_joint
    if extra_frames_urdf:
        macrofile_content = macrofile_content.replace("<!-- #EXTRAFRAMES -->", extra_frames_urdf)
        macro_frames_only_filecontent = macro_frames_only_filecontent.replace("<!-- #EXTRAFRAMES -->", extra_frames_urdf)
    macrofile.write(macrofile_content)
    macro_frames_only_file.write(macro_frames_only_filecontent)
    rospy.loginfo("Wrote " + os.path.join(out_dir, partname+"_macro.urdf"))
    rospy.loginfo("Wrote " + os.path.join(out_dir, partname+"_frames_only_macro.urdf"))





