#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from o2as_fastener_gripper.srv import *
#from dynamixel_workbench_msgs.srv import *

class FastenerGripperController:
    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.fasten = rospy.Service('~Fasten', Fasten, self.handle_fasten)
        self.stop = rospy.Service('~Stop', Fasten, self.handle_stop)
        self.dynamixel_command_write = rospy.ServiceProxy('dynamixel_write_command', DynamixelWriteCommand)
        self.dynamixel_read_state = rospy.ServiceProxy('dynamixel_read_state', DynamixelReadState)

        self.set_torque_limit(256) # half of max
        self.torque_enable(0) # disable
        self.set_control_mode(1) # wheel mode

        rospy.spin()

    def torque_enable(self, value):
        res = self.dynamixel_read_state(self.motor_id, "Torque_Enable")
        if res.value != value:
            self.dynamixel_command_write(self.motor_id, "Torque_Enable", value)

    def set_control_mode(self, value):
        res = self.dynamixel_read_state(self.motor_id, "Control_Mode")
        if res.value != value:
            self.dynamixel_command_write(self.motor_id, "Control_Mode", value)

    def set_torque_limit(self, value):
        res = self.dynamixel_read_state(self.motor_id, "Torque_Limit")
        if res.value != value:
            self.dynamixel_command_write(self.motor_id, "Torque_Limit", value)

    def get_present_speed(self):
        res = self.dynamixel_read_state(self.motor_id, "Present_Speed")
        rospy.loginfo("present_speed = {0}".format(res.value))
        return res.value

    def handle_fasten(self, req):
        rospy.loginfo("call fasten")
        self.dynamixel_command_write(self.motor_id, "Moving_Speed", 1000)
        rospy.sleep(0.5)

        while not rospy.is_shutdown():
            speed = self.get_present_speed()
            if speed < 20:
                break
            rospy.sleep(0.1)

        self.dynamixel_command_write(self.motor_id, "Moving_Speed", 0)
        return 0

    def handle_stop(self, req):
        rospy.loginfo("call stop")
        self.dynamixel_command_write(self.motor_id, "Moving_Speed", 0)
        return 0

if __name__ == '__main__':
    rospy.init_node('fastener_gripper_controller')
    motor_id = rospy.get_param("~motor_id")
    try:
        controller = FastenerGripperController(motor_id)
    except rospy.ROSInterruptException: pass

