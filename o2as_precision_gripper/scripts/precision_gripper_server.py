#!/usr/bin/env python

import lib_robotis_xm430 as xm430
import sys
import time
import rospy
from o2as_msgs.srv import *

class PrecisionGripper:
    def __init__(self, serial_port = '/dev/ttyUSB1'):
        self.dynamixel = xm430.USB2Dynamixel_Device( serial_port, baudrate = 57600 )
        self.p1 = xm430.Robotis_Servo2( self.dynamixel, 1, series = "XM" )  #inner gripper
        self.p2 = xm430.Robotis_Servo2( self.dynamixel, 2, series = "XM" )  #outer gripper

        name = rospy.get_name()
        self.outer_force = rospy.get_param(name + "/outer_force", 30)
        self.inner_force = rospy.get_param(name + "/inner_force", 5)
        rospy.logwarn("inner_force = " + str(self.inner_force))
        self.outer_open_position = rospy.get_param(name + "/outer_open_position", -50000)
        self.outer_close_position = rospy.get_param(name + "/outer_close_position", 50240)
        self.speed_limit = rospy.get_param(name + "/speed_limit", 10)
        return

    def my_callback(self, req):
        rospy.loginfo("Precision gripper callback has been called")
        res = PrecisionGripperCommandResponse()

        if req.stop:
            self.inner_gripper_read_current_position()
            self.inner_gripper_disable_torque()
            self.outer_gripper_disable_torque()
        elif req.open_outer_gripper_fully:
            rospy.loginfo("Opening outer gripper")
            self.outer_gripper_open_fully(self.outer_force)
        elif req.close_outer_gripper_fully:
            rospy.loginfo("Closing outer gripper")
            self.outer_gripper_close_fully(self.outer_force)
        elif req.open_inner_gripper_fully:
            rospy.loginfo("Opening inner gripper")
            self.inner_gripper_open_fully(self.inner_force)
        elif req.close_inner_gripper_fully:
            rospy.loginfo("Closing inner gripper")
            self.inner_gripper_close_fully(self.inner_force)
        else:
            rospy.logerr('No command sent to the gripper, service request was empty.')
            res.success = False
            return res

        res.success = True
        rospy.loginfo("Precision gripper service callback returns.")
        return res

    #outer gripper related functions
    def outer_gripper_close_new(self, goal_position):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(100) #0.3Nm
            i2 = self.p2.read_current_position()
            i_avg = int(i2)
            while i_avg < goal_position:
                self.p2.set_goal_position(i_avg)
                current2 = int(self.p2.read_current())
                if current2 > 100:
                    break
                i_avg = i_avg + 8
        except:
            rospy.logerr("Failed to run commands.")

    def outer_gripper_read_current_position(self):
        try:
            x=self.p2.read_current_position()
            rospy.loginfo("id2="+str(x))
        except:
            rospy.logerr("Failed to run commands.")


    def outer_gripper_open_fully(self,current):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(current)
            self.p2.set_goal_position(self.outer_open_position)
            rospy.sleep(0.1)
            current_velocity = self.p2.read_current_velocity()
            while current_velocity>self.speed_limit:
                print current_velocity
                current_velocity = self.p2.read_current_velocity()
                rospy.sleep(0.1)
        except:
            rospy.logerr("Failed to run commands.")


    def outer_gripper_close_fully(self,current):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(current)
            self.p2.set_goal_position(self.outer_close_position)
            rospy.sleep(0.1)
            current_velocity = self.p2.read_current_velocity()
            while current_velocity>self.speed_limit:
                print current_velocity
                current_velocity = self.p2.read_current_velocity()
                rospy.sleep(0.1)
        except:
            rospy.logerr("Failed to run commands.")

    def outer_gripper_disable_torque(self):
        try:
            self.p2.disable_torque()
        except:
            rospy.logerr("Failed to run commands.")


    def outer_gripper_open_to(self, location):#still editing
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(100)
            self.p2.set_goal_position(location)
        except:
            rospy.logerr("Failed to run commands.")


    #inner gripper related functions
    ######################################################
    ######################################################

    def inner_gripper_read_current_position(self):
        try:
            x=self.p1.read_current_position()
            rospy.loginfo("id1="+str(x))
        except:
            rospy.logerr("Failed to run commands.")

    def inner_gripper_open_fully(self, current):
        try:
            self.p1.set_operating_mode("currentposition")
            self.p1.set_positive_direction("ccw")
            self.p1.set_current(current)
            self.p1.set_goal_position(3799)
            rospy.sleep(0.1)
            current_velocity = self.p1.read_current_velocity()
            while current_velocity>self.speed_limit:
                print current_velocity
                current_velocity = self.p1.read_current_velocity()
                rospy.sleep(0.1)
        except:
            rospy.logerr("Failed to run commands.")

    def inner_gripper_close_fully(self, current):
        try:
            self.p1.set_operating_mode("current")
            self.p1.set_positive_direction("cw")
            self.p1.set_current(current)
            rospy.sleep(0.1)
            current_velocity = self.p1.read_current_velocity()
            while current_velocity>self.speed_limit:
                print current_velocity
                current_velocity = self.p1.read_current_velocity()
                rospy.sleep(0.1)
        except:
            rospy.logerr("Failed to run commands.")

    def inner_gripper_open_slightly(self, current_position):
        try:
            self.p1.set_operating_mode("currentposition")
            self.p1.set_current(8)
            #current_position=self.p1.read_current_position()
            #current_position = current_position-130
            self.p1.set_goal_position(current_position)
        except:
            rospy.logerr("Failed to run commands.")

    def inner_gripper_disable_torque(self):
        try:
            self.p1.disable_torque()
        except:
            rospy.logerr("Failed to run commands.")

if __name__ == "__main__":
    #initialise the class here
    
    rospy.init_node("precision_gripper_server")
    serial_port = rospy.get_param("precision_gripper_server/serial_port", "/dev/ttyUSB2")
    rospy.loginfo("Starting up on serial port: " + serial_port)
    gripper = PrecisionGripper(serial_port)

    my_service = rospy.Service('precision_gripper_command', PrecisionGripperCommand, gripper.my_callback)
    rospy.loginfo("Service precision_gripper is ready")
    rospy.spin()
