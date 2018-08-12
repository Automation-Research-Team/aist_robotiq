import lib_robotis_xm430 as xm430
import sys
import time
import rospy
from o2as_precision_gripper.srv import *

class PrecisionGripper:
    def __init__(self, serial_port = '/dev/ttyUSB0'):

        self.dynamixel1 = xm430.USB2Dynamixel_Device( serial_port, baudrate = 57600 )
        self.p1 = xm430.Robotis_Servo2( self.dynamixel1, 1, series = "XM" )  #inner gripper
        self.p2 = xm430.Robotis_Servo2( self.dynamixel1, 2, series = "XM" )  #outer gripper
        return

    def my_callback(self, req):
        rospy.loginfo("Gripper callback has been called")

        if req.stop:
            self.inner_gripper_disable_torque()
            self.outer_gripper_disable_torque()
        elif req.open_outer_gripper_fully and not(req.close_outer_gripper_fully):
            self.outer_gripper_open_fully()
        elif not(req.open_outer_gripper_fully) and (req.close_outer_gripper_fully):
            self.outer_gripper_close_fully()
        else:
            rospy.logerr('No command sent to the gripper, service request was empty.')
            return False

        res = PrecisionGripperCommandResponse()
        res.success = True
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


    def inner_innergripper_read_current_position(self):
        try:
            x=self.p1.read_current_position()
            rospy.loginfo("id1="+str(x))
        except:
            rospy.logerr("Failed to run commands.")

    def outer_gripper_read_current_position(self):
        try:
            x=self.p2.read_current_position()
            rospy.loginfo("id2="+str(x))
        except:
            rospy.logerr("Failed to run commands.")


    def outer_gripper_open_fully(self):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(30)
            self.p2.set_goal_position(-30000)
        except:
            rospy.logerr("Failed to run commands.")


    def outer_gripper_close_fully(self):
        try:
            self.p2.set_operating_mode("currentposition")
            self.p2.set_current(20)
            self.p2.set_goal_position(70240)
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

    def inner_gripper_open_fully(self, current):
        try:
            self.p1.set_operating_mode("currentposition")
            self.p1.set_current(current)
            self.p1.set_goal_position(0)
        except:
            rospy.logerr("Failed to run commands.")

    def inner_gripper_close_fully(self, current):
        try:
            self.p1.set_operating_mode("currentposition")
            self.p1.set_current(current)
            self.p1.set_goal_position(5000)
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

    def inner_gripper_close_force(self):
        try:
            self.p1.set_operating_mode("current")
            self.p1.set_current(100)
        except:
            rospy.logerr("Failed to run commands.")

    def inner_gripper_disable_torque(self):
        try:
            self.p1.disable_torque()
        except:
            rospy.logerr("Failed to run commands.")