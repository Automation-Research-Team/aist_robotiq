#! /usr/bin/env python

import rospy
import actionlib
import actionlib_tutorials.msg
import os.path
import yaml
from std_msgs.msg import String
from o2as_fastening_tools.srv import *
from o2as_msgs.msg import *
from util import *

class FasteningToolController(object):
    _feedback = FastenerGripperControlFeedback()
    _result = FastenerGripperControlResult()

    def __init__(self):
        config_dir = rospy.get_param("~config_dir")
        config_file = rospy.get_param("~fastening_tools")

        # get data for .yaml
        conf_gripper_filename = config_dir + "/" + config_file + ".yaml"
        fastening_tools = read_object_yaml_config(conf_gripper_filename)

        # initialize motor id table
        self.fastening_tools = dict()
        data_list = fastening_tools['fastening_tools']
        for data in data_list:
            self.fastening_tools.update({data['name'] : data['motor_id']})
        
        self.dynamixel_command_write = rospy.ServiceProxy('o2as_fastening_tools/dynamixel_write_command', DynamixelWriteCommand)
        self.dynamixel_read_state = rospy.ServiceProxy('o2as_fastening_tools/dynamixel_read_state', DynamixelReadState)

        self._action_name = 'o2as_fastening_tools/fastener_gripper_control_action'
        self._as = actionlib.SimpleActionServer(self._action_name, FastenerGripperControlAction, execute_cb=self.execute_control, auto_start = False)
        self._as.start()

    def set_torque_enable(self, motor_id, value):
        res = self.dynamixel_command_write(motor_id, "Torque_Enable", value)
        if res.comm_result:
            return True
        else:
            rospy.logerr('Can not set torque_enable to XL-320. (ID=%i)' %motor_id)
            return False

    def set_moving_speed(self, motor_id, value):
        res = self.dynamixel_command_write(motor_id, "Moving_Speed", value)
        if res.comm_result:
            return True
        else:
            rospy.logerr('Can not set speed to XL-320. (ID=%i)' %motor_id)
            return False

    def get_present_speed(self, motor_id):
        res = self.dynamixel_read_state(motor_id, "Present_Speed")
        if res.comm_result:
            return res.value
        else:
            rospy.logerr('Can not get speed for XL-320. (ID=%i)' %motor_id)
            return -1

    def execute_control(self, goal):
        t_duration = 1
        motor_id = self.fastening_tools[goal.fastening_tool_name]
        self._result.control_result = True
        self._feedback.motor_speed = goal.speed

        rospy.wait_for_service('o2as_fastening_tools/dynamixel_write_command')
        rospy.wait_for_service('o2as_fastening_tools/dynamixel_read_state')

        if goal.direction == "loosen" :
            t_duration = goal.duration
            if t_duration == 0.0 :
                t_duration = 1
            goal.speed = 1024 + goal.speed
            if goal.speed > 2047 :
                goal.speed = 2047
        elif goal.direction == "tighten" :
            if goal.speed > 1023 :
                goal.speed = 1023

        if (goal.fastening_tool_name in self.fastening_tools) == False :
            rospy.logerr("'%s' is not exist in %s." % (goal.fastening_tool_name, self.conf_gripper_filename))
            self._result.control_result = False
            self._as.set_succeeded(self._result)
            return

        if not self.set_moving_speed(motor_id, goal.speed) :
            print('')
            self._result.control_result = False
            self._as.set_succeeded(self._result)
            return

        rospy.sleep(t_duration)

        # process for loosen
        if goal.direction == "loosen" :
            if not self.set_moving_speed(motor_id, 0) :
                self.set_torque_enable(motor_id, 0)
                self._result.control_result = True
                self._as.set_succeeded(self._result)
            else :
                self.set_torque_enable(motor_id, 0)
                self._result.control_result = False
                self._as.set_succeeded(self._result)
            return

        # process for tighten
        while self._feedback.motor_speed > 0:
            if self._as.is_preempt_requested():
                self._as.set_preempted()
                self._result.control_result = False
                break

            first_speed = self.get_present_speed(motor_id)

            while first_speed > 1023 :
                first_speed = self.get_present_speed(motor_id)

            if first_speed == -1 :
                self.set_torque_enable(motor_id, 0)
                self._result.control_result = False
                self._as.set_succeeded(self._result)
                return 
            
            current_speed = first_speed

            rospy.sleep(0.1)

            second_speed = 9999
            while second_speed > 1023 :
                second_speed = self.get_present_speed(motor_id)

            if second_speed == -1 :
                self.set_torque_enable(motor_id, 0)
                self._result.control_result = False
                self._as.set_succeeded(self._result)
                return 

            if first_speed <= 0 and second_speed <=0:
                current_speed = 0
            elif second_speed > 0 :
                current_speed = second_speed

            self._feedback.motor_speed = current_speed
            self._as.publish_feedback(self._feedback)

        if not self.set_moving_speed(motor_id, 0) :
            self.set_torque_enable(motor_id, 0)
            self._result.control_result = False
            self._as.set_succeeded(self._result)
            return

        if not self.set_torque_enable(motor_id, 0) :
            self._result.control_result = False
            self._as.set_succeeded(self._result)
            return
            
        if self._result.control_result :
            self._feedback.motor_speed = 0
            self._as.publish_feedback(self._feedback)
            
            self._as.set_succeeded(self._result)

        
if __name__ == '__main__':
    rospy.init_node('fastening_tool_controller')
    server = FasteningToolController()
    rospy.spin()
