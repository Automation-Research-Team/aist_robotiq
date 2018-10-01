#! /usr/bin/env python

import rospy
import actionlib
import actionlib_tutorials.msg
from o2as_fastening_tools.srv import *
from o2as_msgs.msg import *
from ur_msgs.msg import *
from ur_msgs.srv import *
from util import *
from std_msgs.msg import String, Bool

class SuctionController(object):
    _feedback = SuctionControlFeedback()

    def __init__(self):
        self.comm_result = False
        self.pin_state = False
        config_dir = rospy.get_param("~config_dir")
        config_file = rospy.get_param("~suction_control")
        
        rospy.Subscriber("b_bot_controller/ur_driver/io_states", IOStates, self.io_state_callback, queue_size=1)
        self.pub = rospy.Publisher('o2as_fastening_tools/screw_suctioned', PressureSensorState, queue_size=18)
        
        self.set_io = rospy.ServiceProxy('b_bot_controller/ur_driver/set_io', SetIO)

        # get data for .yaml
        conf_suction_filename = config_dir + "/" + config_file + ".yaml"
        conf_file_content = read_object_yaml_config(conf_suction_filename)

        # initialize ur_control table
        self.digital_in_port = dict()
        self.digital_out_port_vac = dict()
        self.digital_out_port_blow = dict()
        self.in_state = dict()
        self.out_state = dict()
        self.tool_suction_publisher = dict()

        # get data for .yaml
        self.suction_tool_list = conf_file_content['suction_control']
        for tool_data in self.suction_tool_list:
            self.digital_in_port.update({tool_data['name']: tool_data['digital_in_port']})
            self.digital_out_port_vac.update({tool_data['name']: tool_data['digital_out_port_vac']})
            self.digital_out_port_blow.update({tool_data['name']: tool_data['digital_out_port_blow']})
            self.tool_suction_publisher[tool_data['name']] = rospy.Publisher(tool_data['name'] + '/screw_suctioned', Bool, queue_size=1)
            # Goal: Publish a boolean for each tool under '[tool_name]/screw_suctioned'

        self._action_name = 'o2as_fastening_tools/suction_control'
        self._as = actionlib.SimpleActionServer(self._action_name, SuctionControlAction, execute_cb=self.suction_control, auto_start = False)
        self._as.start()
    
    def io_state_callback(self, data):
        for read_in_status in data.digital_in_states:
            self.in_state.update({read_in_status.pin: read_in_status.state})

        for read_out_status in data.digital_out_states:
            self.out_state.update({read_out_status.pin: read_out_status.state})

        for tool_data in self.suction_tool_list:
            bool_msg = Bool
            bool_msg.data = self.in_state[ self.digital_in_port[tool_data['name']] ]
            print(tool_data['name'])
            print(bool_msg)
            print("----")
            print(bool_msg.data)
            print("====")
            self.tool_suction_publisher[tool_data['name']].publish(bool_msg)

    def set_out_pin_switch(self, port, state):
        success_flag = True

        if state > 0:
            b_state = True
        else:
            b_state = False

        start_time = rospy.get_rostime()

        while ((rospy.get_rostime().secs - start_time.secs) <= 3.0):
            res = self.set_io(1, port, state)
            if not res:
                success_flag = False
                break
            if (port in self.out_state):
                if self.out_state[port] == b_state:
                    break

        if success_flag:
            rospy.loginfo("Success. Digital_out_port(%d) changed %r." % (port, b_state))
        else:
            rospy.logerr("Error. Digital_out_port(%d) can't be changed." % (port))

        return success_flag
        
        
    def suction_control(self, goal):
        success_flag = True
        res = SuctionControlResult()

        # yaml file check
        if (goal.fastening_tool_name in self.digital_in_port) == False:
            rospy.logerr("yaml file: '%s' does not exist in %s." % (goal.fastening_tool_name, self.digital_in_port))
            success_flag = False

        if (goal.fastening_tool_name in self.digital_out_port_vac) == False:
            rospy.logerr("yaml file: '%s' does not exist in %s." % (goal.fastening_tool_name, self.digital_out_port_vac))
            success_flag = False

        if (goal.fastening_tool_name in self.digital_out_port_blow) == False:
            rospy.logerr("yaml file: '%s' does not exist in %s." % (goal.fastening_tool_name, self.digital_out_port_blow))
            success_flag = False
            
        if not success_flag:
            res.success = success_flag
            self._as.set_aborted(res)
            return

        vac_port = self.digital_out_port_vac[goal.fastening_tool_name]
        blow_port = self.digital_out_port_blow[goal.fastening_tool_name]
        
        if goal.turn_suction_on:
            if not self.set_out_pin_switch(blow_port, 0):
                success_flag = False
            if not self.set_out_pin_switch(vac_port, 1):
                success_flag = False
        elif goal.drop_screw:
            if not self.set_out_pin_switch(vac_port, 0):
                success_flag = False
            if not self.set_out_pin_switch(blow_port, 1):
                success_flag = False
        else: # turn_suction_on and pick_screw == False
            if not self.set_out_pin_switch(blow_port, 0):
                success_flag = False
            if not self.set_out_pin_switch(vac_port, 0):
                success_flag = False

        if not success_flag:
            if not self.set_out_pin_switch(blow_port, 0):
                success_flag = False
            if not self.set_out_pin_switch(vac_port, 0):
                success_flag = False
        
        res.success = success_flag
        if not success_flag:
            self._as.set_aborted(res)
        self._as.set_succeeded(res)
        return


if __name__ == '__main__':
    rospy.init_node('suction_controller')
    server = SuctionController()
    rospy.spin()
