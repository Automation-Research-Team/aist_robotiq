#! /usr/bin/env python
import lib_robotis_xm430 as xm430
import sys
import time
import rospy
import actionlib
import o2as_msgs.msg
import o2as_msgs.srv

class ToolsAction:
    def __init__(self):
        name = rospy.get_name()
        serial_port = rospy.get_param(name + "/serial_port", "/dev/ttyUSB1")
        rospy.loginfo("Starting up on serial port: " + serial_port)
        self.dynamixel = xm430.USB2Dynamixel_Device( serial_port, baudrate = 57600 )
        self.p1 = xm430.Robotis_Servo2( self.dynamixel, 1, series = "XM" )  #Peg
        self.p2 = xm430.Robotis_Servo2( self.dynamixel, 2, series = "XM" )  #Big nut
        self.p3 = xm430.Robotis_Servo2( self.dynamixel, 3, series = "XM" )  #small nut


        #define the action
        self._action_name = "tools_action"
        self._action_server = actionlib.SimpleActionServer(self._action_name, o2as_msgs.msg.ToolsCommandAction, execute_cb=self.action_callback, auto_start = False)
        self._action_server.start()
        rospy.loginfo('Action server '+ str(self._action_name)+" started.")

        return

    def action_callback(self, goal):
        # publish info to the console for the user
        rospy.loginfo('Executing'+ str(self._action_name)+"."+"request sent:")
        rospy.loginfo(goal)

        # start executing the action
        command_is_sent = False
        if goal.stop:
            rospy.loginfo("Turning off torque.")
            command_is_sent1 = self.peg_disable_torque()
            command_is_sent2 = self.big_nut_disable_torque()
            command_is_sent3 = self.small_nut_disable_torque()
            
            if command_is_sent1 and command_is_sent2 and command_is_sent3 is True:
                command_is_sent = True
            else:
                command_is_sent = False
                
        elif goal.peg_fasten:        
            command_is_sent = self.peg_fasten(30)
            
        elif goal.big_nut_fasten:
            command_is_sent = self.big_nut_fasten(30)
            
        elif goal.small_nut_fasten:
            command_is_sent = self.small_nut_fasten(30)

        else:
            rospy.logerr('No command is sent, service request was empty.')
            command_is_sent = False
        
        success = command_is_sent
        if success:
            if goal.stop:
                self._feedback.motor_speed = -1 #an arbitary number higher than self.speed_limit
  
            elif goal.peg_fasten or goal.big_nut_fasten or goal.small_nut_fasten:  
                if goal.peg_fasten:
                    self._feedback.motor_speed = self.p1.read_current_velocity()
                elif goal.big_nut_fasten:
                    self._feedback.motor_speed = self.p2.read_current_velocity()
                elif goal.small_nut_fasten:
                    self._feedback.motor_speed = self.p3.read_current_velocity()
             
            countTime = 0
            while self._feedback.motor_speed > self.speed_limit or countTime > 50:
                rospy.sleep(0.1)
                countTime++
                # check that preempt has not been requested by the client
                if self._action_server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self._action_server.set_preempted()
                    success = False
                    break
                    
                if goal.peg_fasten or goal.big_nut_fasten or goal.small_nut_fasten:  
                    if goal.peg_fasten:
                        self._feedback.motor_speed = self.p1.read_current_velocity()
                    elif goal.big_nut_fasten:
                        self._feedback.motor_speed = self.p2.read_current_velocity()
                    elif goal.small_nut_fasten:
                        self._feedback.motor_speed = self.p3.read_current_velocity()
                        
                # publish the feedback
                self._action_server.publish_feedback(self._feedback)
                
            if success:
                self._result.success = True
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self._action_server.set_succeeded(self._result)
        else:
            self._action_server.set_preempted()

  
    ######################################################

    def peg_fasten(self, current):
        try:
            self.p1.set_operating_mode("current")
            self.p1.set_positive_direction("cw")
            self.p1.set_current(current)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def big_nut_fasten(self, current):
        try:
            self.p2.set_operating_mode("current")
            self.p2.set_positive_direction("cw")
            self.p2.set_current(current)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False
            
    def small_nut_fasten(self, current):
        try:
            self.p3.set_operating_mode("current")
            self.p3.set_positive_direction("cw")
            self.p3.set_current(current)
            rospy.sleep(0.1)
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

    def peg_disable_torque(self):
        try:
            self.p1.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False
            
    def big_nut_disable_torque(self):
        try:
            self.p2.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False
            
    def small_nut_disable_torque(self):
        try:
            self.p3.disable_torque()
            return True
        except:
            rospy.logerr("Failed to run commands.")
            return False

        
if __name__ == '__main__':
    rospy.init_node('tools_server')
    server = ToolsAction()
    rospy.spin()
