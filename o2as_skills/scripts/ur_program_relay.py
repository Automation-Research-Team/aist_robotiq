#!/usr/bin/env python

import roslib
import rospy
# roslib.load_manifest('ur_program_relay')

import moveit_msgs.msg
import std_msgs.msg
import o2as_msgs.srv

import os, sys, rospkg


# Node example class.
class URScriptRelay():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.publishers = {
            'a_bot':rospy.Publisher("/a_bot_controller/ur_driver/URScript", std_msgs.msg.String, queue_size=1), 
            'b_bot':rospy.Publisher("/b_bot_controller/ur_driver/URScript", std_msgs.msg.String, queue_size=1), 
            'c_bot':rospy.Publisher("/c_bot_controller/ur_driver/URScript", std_msgs.msg.String, queue_size=1) }
        rospy.sleep(0.5) # Wait for the publishers to be registered
        
        self.rospack = rospkg.RosPack()
        self.read_templates()
        s = rospy.Service('o2as_skills/sendScriptToUR', o2as_msgs.srv.sendScriptToUR, self.srv_callback)

        # rospy.loginfo("TESTING SCRIPT SENDING")
        # rospack = rospkg.RosPack()
        # program = ""
        # program_template = open(os.path.join(rospack.get_path("o2as_examples"), "scripts/ur", "move_back_forth_5cm.script"), 'rb')
        # program_line = program_template.read(1024)
        # while program_line:
        #     program += program_line
        #     program_line = program_template.read(1024)

        # # Send the program to the robot
        # program_msg = std_msgs.msg.String()
        # program_msg.data = program
        # self.publishers["b_bot"].publish(program_msg)
        # rospy.loginfo("SENT")
        
        # Main while loop.
        while not rospy.is_shutdown():
            rospy.sleep(.1)

    # Create a callback function for the service.
    def srv_callback(self, req):
        # Interpret the service parameters, construct the program, send it to the UR
        if not req.robot_name:
            rospy.logerr("robot_name was not defined in the service call to sendScriptToUR!")
            return False
        elif not req.program_id:
            rospy.logerr("No program ID was defined!")
            return False

        if req.program_id == "insertion":
            program_front = self.insertion_template
            program_mid = ""

            # Assign defaults
            if not req.force_magnitude:
                req.force_magnitude = 5.0
            if not req.force_direction:
                req.force_direction = "Z+"
            if not req.forward_speed:
                req.forward_speed = .02
            if not req.max_approach_distance:
                req.max_approach_distance = .02
            if not req.max_radius:
                req.max_radius = 4.0
            if not req.radius_increment:
                req.radius_increment = 0.3
            if not req.peck_mode:
                req.peck_mode = False
            if not req.stroke:
                req.stroke = 0.035
            if not req.impedance_mass:
                req.impedance_mass = 10

            # Function definitions:
            # rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )
            # rq_spiral_search_new(stroke, force_threshold = 3, max_radius = 5.0, radius_incr=0.3, peck_mode = False):

            program_mid += "        textmsg(\"Approaching.\")\n"
            program_mid += "        rq_linear_search(\"" + req.force_direction + "\"," \
                                + str(req.force_magnitude) + "," \
                                + str(req.forward_speed) + "," \
                                + str(req.max_approach_distance) + ")\n"
            program_mid += "        stroke = " + str(req.stroke) + "\n"
            program_mid += "        textmsg(\"Spiral searching.\")\n"
            program_mid += "        if rq_spiral_search_new(stroke," + str(req.force_magnitude) \
                                + ", " + str(req.max_radius) \
                                + ", " + str(req.radius_increment) \
                                + ", peck_mode=" + str(req.peck_mode) + "):\n"
            program_mid += "            #Insert the Part into the bore#\n"
            program_mid += "            textmsg(\"Impedance insert\")\n"
            program_mid += "            rq_impedance(stroke, " + str(req.impedance_mass) + ")\n"
            program_mid += "        end\n"
            program_mid += "    end\n"
            program_mid += "end\n"

            program = program_front + "\n" + program_mid # + "\n" + program_back
        elif req.program_id == "lin_move":
            rospy.logerr("LIN MOVE IS NOT IMPLEMENTED YET") 
            # TODO: Transform the pose to the robot base coordinates
            # TODO: 
            # TODO: Send the pose to the robot
            # program = ""
            # program += "def move_to_pose_lin():\n"
            # program += "    textmsg(\"Moving to a pose.\")\n"
            # program += "    target_pos=p[0.0, 0.0, 0.05, 0.0, 0.0, 0.0]\n"
            # program += "    movel(pose_trans(p[0.0,0.0,0.0,0.0,0.0,0.0], target_pos), a=0.5, v=0.1)\n"
            # program += "end\n"
        elif req.program_id == "spiral_press":
            rospy.logerr("SPIRAL PRESS IS NOT IMPLEMENTED YET") # TODO
        elif req.program_id == "test":
            program = ""
            program_file = open(os.path.join(self.rospack.get_path("o2as_examples"), "scripts/ur", "move_back_forth_5cm.script"), 'rb')
            program_line = program_file.read(1024)
            while program_line:
                program += program_line
                program_line = program_file.read(1024)

        # Send the program to the robot
        program_msg = std_msgs.msg.String()
        program_msg.data = program

        rospy.loginfo("Sending UR robot command.")
        rospy.loginfo("Program is:")
        rospy.loginfo(program)
        self.publishers[req.robot_name].publish(program_msg)
        return True

    def read_templates(self):
        # Read the files containing the program templates into memory
        self.insertion_template = ""
        program_template_file = open(os.path.join(self.rospack.get_path("o2as_skills"), "src/urscript", "peginholespiral_imp_osx.script"), 'rb')
        program_line = program_template_file.read(1024)
        linecounter = 0
        while program_line:
            self.insertion_template += program_line
            program_line = program_template_file.read(1024)
        return True


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('urscript_construction_node')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = URScriptRelay()
    except rospy.ROSInterruptException: pass
