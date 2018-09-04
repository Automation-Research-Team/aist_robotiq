#! /usr/bin/env python

import rospy
import os.path
from std_msgs.msg import String
from o2as_fastening_tools.srv import *
from o2as_fastening_tools.msg import *

class FasteningToolController(object):
    _feedback = FastenerGripperControlFeedback()
    _result = FastenerGripperControlResult()

    def __init__(self):
        self.dynamixel_command_write = rospy.ServiceProxy('dynamixel_write_command', DynamixelWriteCommand)

    def set_motor_id(self, motor_id, value):
        res = self.dynamixel_command_write(motor_id, "ID", value)
        if res.comm_result:
            rospy.loginfo('Changed the ID of the motor (ID=%i)' %value)
        else:
            rospy.logerr('Can not set ID to XL-320 (ID=%i)' %motor_id)

        
if __name__ == '__main__':
    rospy.init_node('motor_id_setup')
    server = FasteningToolController()

    rospy.sleep(5)

    while not rospy.is_shutdown() :
        try:
            rospy.loginfo('Set Motor ID.')
            rospy.loginfo('Please enter the original ID, then enter the new ID.')
            rospy.loginfo('The delimiter is space.')
            old_new_id = map(int, raw_input().split())
            break
        except:
            rospy.logerr('Input error.')
            rospy.logerr('Please input again.')
    
    server.set_motor_id(old_new_id[0],old_new_id[1])
