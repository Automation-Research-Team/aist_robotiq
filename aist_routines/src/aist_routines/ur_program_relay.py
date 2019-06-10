#!/usr/bin/env python

# This program is based on o2as_skills/scripts/ur_program_relay.py

import roslib
import rospy
# roslib.load_manifest('ur_program_relay')

import moveit_msgs.msg
import std_msgs.msg
import o2as_msgs.srv
import tf

import os, sys, rospkg


# Node example class.
class URScriptRelay():
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        self.listener = tf.TransformListener()
        self.publishers = {
            'a_bot': rospy.Publisher("/a_bot_controller/ur_driver/URScript",
                                     std_msgs.msg.String, queue_size=1),
            'b_bot': rospy.Publisher("/b_bot_controller/ur_driver/URScript",
                                     std_msgs.msg.String, queue_size=1),
            'c_bot': rospy.Publisher("/c_bot_controller/ur_driver/URScript",
                                     std_msgs.msg.String, queue_size=1),
            'd_bot': rospy.Publisher("/d_bot_controller/ur_driver/URScript",
                                     std_msgs.msg.String, queue_size=1),
        }

         # Wait for the publishers to be registered and the listener to receive
        rospy.sleep(0.5)

        self.rospack = rospkg.RosPack()
        self.read_templates()
        s = rospy.Service('aist_skills/sendScriptToUR',
                          o2as_msgs.srv.sendScriptToUR, self.srv_callback)

        # Main while loop.
        while not rospy.is_shutdown():
            rospy.sleep(.1)

    # Create a callback function for the service.
    def srv_callback(self, req):
        # Interpret the service parameters, construct the program,
        # send it to the UR
        if not req.robot_name:
            rospy.logerr("robot_name was not defined in the service call to sendScriptToUR!")
            return False
        elif not req.program_id:
            rospy.logerr("No program ID was defined!")
            return False

        if req.program_id == "linear_push":
            program_front = self.linear_push_template
            program_back = ""

            # Assign defaults
            if not req.max_force:
                req.max_force = 10.0
            if not req.force_direction:
                req.force_direction = "Z+"
            if not req.forward_speed:
                req.forward_speed = .02
            if not req.max_approach_distance:
                req.max_approach_distance = .1

            ### Function definitions, for reference:
            ### rq_linear_search(direction="Z+",force = 10, speed = 0.004, max_distance = 0.02 )

            program_back += "    rq_zero_sensor()\n"
            program_back += "    textmsg(\"Approaching linearly.\")\n"
            program_back += "    rq_linear_search(\"" + req.force_direction + "\"," \
                                + str(req.max_force) + "," \
                                + str(req.forward_speed) + "," \
                                + str(req.max_approach_distance) + ")\n"
            program_back += "    textmsg(\"Done.\")\n"
            program_back += "end\n"

            program = program_front + "\n" + program_back

        # Send the program to the robot
        program_msg = std_msgs.msg.String()
        program_msg.data = program

        rospy.loginfo("Sending UR robot program " + req.program_id)
        # rospy.logdebug("Program is:")
        # rospy.logdebug(program)
        self.publishers[req.robot_name].publish(program_msg)
        return True

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('o2as_urscript_construction_node')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = URScriptRelay()
    except rospy.ROSInterruptException: pass
