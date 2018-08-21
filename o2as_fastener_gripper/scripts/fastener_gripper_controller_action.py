#! /usr/bin/env python

import rospy
import actionlib
import actionlib_tutorials.msg
from std_msgs.msg import String
from o2as_fastener_gripper.srv import *
from o2as_fastener_gripper.msg import *

class FastenerGripperController(object):
    # create messages that are used to publish feedback/result
    _feedback = FastenerGripperControlFeedback()
    _result = FastenerGripperControlResult()

    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.dynamixel_command_write = rospy.ServiceProxy('dynamixel_write_command', DynamixelWriteCommand)
        self.dynamixel_read_state = rospy.ServiceProxy('dynamixel_read_state', DynamixelReadState)
        
        #Initial Setting(XL320)
        self.torque_enable(0)       # disable
        self.set_control_mode(1)    # wheel mode
        self.set_torque_limit(1023) # half of max

        self._action_name = '~Action'
        self._as = actionlib.SimpleActionServer(self._action_name, FastenerGripperControlAction, execute_cb=self.execute_control, auto_start = False)
        self._as.start()
    
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
        return res.value

    def set_moving_speed(self, value):
        res = self.dynamixel_command_write(self.motor_id, "Moving_Speed", value)
        if res.comm_result:
            return True
        else:
            return False

    def execute_control(self, goal):
        self._result.control_result = True
        self._feedback.motor_speed = rospy.get_param("~speed")

        self.set_moving_speed(self._feedback.motor_speed)
        
        rospy.sleep(1)

        while self._feedback.motor_speed > 0:
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                self._result.control_result = False
                break

            res = self.dynamixel_read_state(self.motor_id, "Present_Speed")

            self._feedback.motor_speed = res.value

            self._as.publish_feedback(self._feedback)
            rospy.sleep(1)

        self.set_moving_speed(0)
        
        if self._result.control_result :
            self._feedback.motor_speed = 0
            self._as.publish_feedback(self._feedback)
        
        self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
    rospy.init_node('fastener_gripper_controller')
    motor_id = rospy.get_param("~motor_id")
    server = FastenerGripperController(motor_id)
    rospy.spin()