#!/usr/bin/env python
import csv
import os

import rospy
import rospkg

rp = rospkg.RosPack()

with open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", "taskboard_measurement.csv"), 'r') as csvfile:
    reader = csv.reader(csvfile)
    # print reader
    header = next(reader)
    
    f = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", 'o2as_taskboard_template_front.xacro'),'r')
    template_front = f.read()
    f.close()
    
    outfile = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf", 'task_board.xacro'),'w+')
    outfile.write(template_front)
    
    for row in reader:
        row[1]=float(row[1])/1000.
        row[2]=float(row[2])/1000.
        row[3]=float(row[3])/1000.

        outfile.write("  <joint name=\"${boardname}_part"+str(row[0])+"\" type=\"fixed\">\n")

        if str(row[0]) == "6_large_pulley":
            outfile.write("    <parent link=\"${boardname}_part6_small_pulley\"/>\n")
        elif str(row[0]) == "7_2":
            outfile.write("    <parent link=\"${boardname}_part7_1\"/>\n")
        else:
            outfile.write("    <parent link=\"${boardname}_surface\"/>\n")
        
        outfile.write("    <child link=\"${boardname}_part"+str(row[0])+"\"/>\n")

        if str(row[4]) == "N.A":
            outfile.write("    <origin rpy=\"0.0 0.0 ${pi/2}\" xyz=\""+str(row[1])+" "+str(row[2])+" "+str(row[3])+"\"/>\n")
        else:
            temp_angle = (90. + float(row[4]))/180.
            outfile.write("    <origin rpy=\"0.0 0.0 ${pi*"+str(temp_angle)+"}\" xyz=\""+str(row[1])+" "+str(row[2])+" "+str(row[3])+"\"/>\n")

        outfile.write("  </joint>\n")

        if str(row[0]) == "1" or str(row[0]) == "2" or str(row[0]) == "3" or str(row[0]) == "4" or str(row[0]) == "7_1" or str(row[0]) == "11" or str(row[0]) == "12" or str(row[0]) == "13" or str(row[0]) == "15":
            outfile.write("  <link name=\"${boardname}_part"+str(row[0])+"\"/>\n\n")
        else:
            f = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates/taskboard_part_template", 'o2as_taskboard_part_'+str(row[0])+'_template_tail.xacro'),'r')
            intermediate_file = f.read()
            f.close()
            outfile.write(intermediate_file)

    f = open(os.path.join(rp.get_path("o2as_scene_description"), "urdf/templates", 'o2as_taskboard_template_tail.xacro'),'r')
    template_tail = f.read()
    f.close()
    outfile.write(template_tail)