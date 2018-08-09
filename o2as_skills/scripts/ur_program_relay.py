#!/usr/bin/env python

import roslib
import rospy
# roslib.load_manifest('ur_program_relay')

import moveit_msgs.msg
import std_msgs.msg
import o2as_skills.srv

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
        s = rospy.Service('o2as_skills/ur_program_relay', o2as_skills.srv.sendScriptToUR, self.srv_callback)


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
            rospy.logerr("robot_name was not defined in the service call to ur_program_relay!")
            return False
        elif not req.program_id:
            rospy.logerr("No program ID was defined!")
            return False

        if req.program_id == "insertion":
            program_front = self.insertion_template_front
            program_back = self.insertion_template_back

            # FUNCTION:  rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )

            # program_mid += "        textmsg(\"Approaching.\")"
            # program_mid += "        rq_linear_search(\"" + req.force_direction + "\"," \
            #                     + str(req.force_magnitude) + "," \
            #                     + str(req.forward_speed) + "," \
            #                     + str(req.max_insertion_distance) + ")"
            # program_mid += "        stroke = 0.035"
            # program_mid += "        textmsg(\"Spiral searching.\")"
            # program_mid += "        if rq_spiral_search_new(stroke," + str(req.force_magnitude) \
            #                     + "," + str(req.max_radius) \
            #                     + ",.3," \   
            #                     + "peck_mode=" + str(req.peck_mode) + "):"
            # program_mid += "            #Insert the Part into the bore#"
            # program_mid += "            textmsg(\"Impedance insert\")"
            # program_mid += "            massm = 10"     # 
            # program_mid += "            rq_impedance(stroke, massm)"
            # program_mid += "        end"

            # The original version, to debug
            program_mid += "        textmsg(\"tst0\")"
            program_mid += "        rq_linear_search(\"Z+\",10,0.02,0.2)"
            program_mid += "        stroke = 0.035"
            program_mid += "        if rq_spiral_search_new(stroke,10,7,.3,peck_mode=True):"
            program_mid += "            #Insert the Part into the bore#"
            program_mid += "            textmsg(\"Impedance insert\")"
            program_mid += "            massm = 10"
            program_mid += "            rq_impedance(stroke, massm)"
            program_mid += "        end"
            program_mid += "        textmsg(\"running\")"
            ####

            program = program_front + program_mid + program_back
        elif req.program_id == "lin_move":
            rospy.logerr("LIN MOVE IS NOT IMPLEMENTED YET") # TODO
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
        self.publishers[req.robot_name].publish(program_msg)
        return True

    def read_templates(self):
        # Read the files containing the program templates into memory
        self.insertion_template_front = ""
        self.insertion_template_back  = ""
        program_template_file = open(os.path.join(self.rospack.get_path("o2as_skills"), "src/urscript", "peginholespiral_imp_short.script"), 'rb')
        program_line = program_template_file.read(1024)
        linecounter = 0
        while program_line:
            linecounter += 1
            if linecounter < 916:
                self.insertion_template_front += program_line
            else:
                self.insertion_template_back += program_line
            program_line = program_template_file.read(1024)
            
        return True


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('urscript_construction_node')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = URScriptRelay()
    except rospy.ROSInterruptException: pass
