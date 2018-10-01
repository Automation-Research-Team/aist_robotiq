#! /usr/bin/env python

import rospy
import actionlib
import actionlib_tutorials.msg
from o2as_fastening_tools.srv import *
from o2as_msgs.msg import *
from ur_msgs.msg import *
from ur_msgs.srv import *
from util import *
from std_msgs.msg import String

class ScrewController(object):
    _feedback = ScrewControlFeedback()
    _result = ScrewControlResult()

    def __init__(self):
        self.comm_result = False
        self.pin_state = False
        config_dir = rospy.get_param("~config_dir")
        config_file = rospy.get_param("~screw_controls")
        
        rospy.Subscriber("b_bot_controller/ur_driver/io_states", IOStates, self.callback, queue_size=1)
        self.pub = rospy.Publisher('o2as_fastening_tools/screw_suctioned', PressureSensoState, queue_size=18)
        
        self.set_io = rospy.ServiceProxy('b_bot_controller/ur_driver/set_io', SetIO)

        # get data for .yaml
        conf_screw_filename = config_dir + "/" + config_file + ".yaml"
        screw_list = read_object_yaml_config(conf_screw_filename)

        # initialize ur_control table
        self.digital_in_port = dict()
        self.digital_out_port_vac = dict()
        self.digital_out_port_blow = dict()
        self.in_state = dict()
        self.out_state = dict()

        # get data for .yaml
        data_list = screw_list['screw_controls']
        for data in data_list:
            self.digital_in_port.update({data['name'] : data['digital_in_port']})
            self.digital_out_port_vac.update({data['name'] : data['digital_out_port_vac']})
            self.digital_out_port_blow.update({data['name'] : data['digital_out_port_blow']})

        self._action_name = 'o2as_fastening_tools/screw_control_action'
        self._as = actionlib.SimpleActionServer(self._action_name, ScrewControlAction, execute_cb=self.execute_control, auto_start = False)
        self._as.start()
    
    def callback(self, data):
        for read_in_status in data.digital_in_states :
            self.in_state.update({read_in_status.pin : read_in_status.state})
            self.pub.publish(read_in_status.pin, read_in_status.state)

        for read_out_status in data.digital_out_states :
            self.out_state.update({read_out_status.pin : read_out_status.state})

    def out_pin_switch(self, port, state) :
        success_flg = True

        if state > 0 :
            b_state = True
        else :
            b_state = False

        start_time = rospy.get_rostime()

        while ((rospy.get_rostime().secs - start_time.secs) <= 3.0) :
            res = self.set_io(1, port, state)
            if not res :
                success_flg = False
                break
            if (port in self.out_state) :
                if self.out_state[port] == b_state :
                    break

        if success_flg :
            rospy.loginfo("Success. Digital_out_port(%d) changed %r." % (port, b_state))
        else :
            rospy.logerr("Error. Digital_out_port(%d) can't be changed." % (port))

        return success_flg
        
        
    def execute_control(self, goal):
        success_flag = True

        # yaml file check
        if (goal.fastening_tool_name in self.digital_in_port) == False :
            rospy.logerr("yaml file : '%s' is not exist in %s." % (goal.fastening_tool_name, self.digital_in_port))
            success_flag = False

        if (goal.fastening_tool_name in self.digital_out_port_vac) == False :
            rospy.logerr("yaml file : '%s' is not exist in %s." % (goal.fastening_tool_name, self.digital_out_port_vac))
            success_flag = False

        if (goal.fastening_tool_name in self.digital_out_port_blow) == False :
            rospy.logerr("yaml file : '%s' is not exist in %s." % (goal.fastening_tool_name, self.digital_out_port_blow))
            success_flag = False
            
        if not success_flag :
            self._result.control_result = success_flag
            self._as.set_succeeded(self._result)
            return

        vac_port = self.digital_out_port_vac[goal.fastening_tool_name]
        blow_port = self.digital_out_port_blow[goal.fastening_tool_name]
        
        if goal.switch :
            pressure_sensor_input = True       # pressure_sensor is Digital_input
            
            if goal.screw_control == "Insert" :
                if not self.out_pin_switch(blow_port, 0) :
                    success_flag = False

                if not self.out_pin_switch(vac_port, 1) :
                    success_flag = False
            
            elif goal.screw_control == "Remove" :
                pressure_sensor_input = False

                if not self.out_pin_switch(vac_port, 0) :
                    success_flag = False

                if not self.out_pin_switch(blow_port, 1) :
                    success_flag = False

            if not success_flag :
                self._result.control_result = success_flag
                self._as.set_succeeded(self._result)
                return

            ur_control_in_port = self.digital_in_port[goal.fastening_tool_name]

            start_time = rospy.get_rostime()

            while ((rospy.get_rostime().secs - start_time.secs) <= 5.0) :
                if ur_control_in_port in self.in_state :
                    self._feedback.screw_status = self.in_state[ur_control_in_port]
                    self._as.publish_feedback(self._feedback)
                        
                    if self.in_state[ur_control_in_port] == pressure_sensor_input :
                        success_flag = True
                        break
                success_flag = False

        else :
            if not self.out_pin_switch(blow_port, 0) :
                success_flag = False

            if not self.out_pin_switch(vac_port, 0) :
                success_flag = False


        if not success_flag :
            if not self.out_pin_switch(blow_port, 0) :
                success_flag = False

            if not self.out_pin_switch(vac_port, 0) :
                success_flag = False

        self._result.control_result = success_flag
        
        self._as.set_succeeded(self._result)
        return


if __name__ == '__main__':
    rospy.init_node('screw_controller')
    server = ScrewController()
    rospy.spin()