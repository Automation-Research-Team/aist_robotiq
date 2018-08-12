import lib_robotis_xm430 as xm430
import sys
import time
import rospy


class PrecisionGripper:
    def __init__(self, serial_port = '/dev/ttyUSB0'):

        self.dynamixel1 = xm430.USB2Dynamixel_Device( serial_port, baudrate = 57600 )
        self.p1 = xm430.Robotis_Servo2( self.dynamixel1, 1, series = "XM" )  #inner gripper
        self.p2 = xm430.Robotis_Servo2( self.dynamixel1, 2, series = "XM" )  #outer gripper
        return

    def my_callback(self, req):
        rospy.loginfo("The Service has been called")

        if req.stop:
            self.inner_gripper_disable_torque()
            self.outer_gripper_disable_torque()
        elif req.open_outer_gripper_fully and not(req.close_outer_gripper_fully):
            self.outer_gripper_open_fully()
        elif not(req.open_outer_gripper_fully) and (req.close_outer_gripper_fully):
            self.outer_gripper_close_fully()
        
        else:
            print('Again!!!!!!!!!!!!!!!!!!!!')

        req.success = True
        return True

    #outer gripper related functions
    def outer_gripper_close_new(self, goal_position):
        try:
            #%%
            self.p2.set_operating_mode("currentposition")
            #%%
            self.p2.set_current(100) #0.3Nm
            i2 = self.p2.read_current_position()
            i_avg = int(i2)
            while i_avg < goal_position:
                self.p2.set_goal_position(i_avg)
                current2 = int(self.p2.read_current())
                if current2 > 100:
                    break
                i_avg = i_avg + 8
            #%%
        except:
            print "Failed to run commands."


    def inner_innergripper_read_current_position(self):
        try:
            x=self.p1.read_current_position()
            print"id1="+str(x)
        except:
            print "Failed to run commands."

    def outer_gripper_read_current_position(self):
        try:
            #%%
            x=self.p2.read_current_position()
            print "id2="+str(x)
        except:
            print "Failed to run commands."


    def outer_gripper_open_fully(self):
        try:
            #%%
            self.p2.set_operating_mode("currentposition")
            #%%
            self.p2.set_current(30)
            #%%
            #%%
            self.p2.set_goal_position(-30000)
            #%%
        except:
            print "Failed to run commands."


    def outer_gripper_close_fully(self):
        try:
            #%%
            self.p2.set_operating_mode("currentposition")
            #%%
            self.p2.set_current(20)
            #%%
            #%%
            self.p2.set_goal_position(70240)
            #%%
        except:
            print "Failed to run commands."

    def outer_gripper_disable_torque(self):
        try:
            #%%
            self.p2.disable_torque()
            #%%
        except:
            print "Failed to run commands."


    def outer_gripper_open_to(self, location):#still editing
        try:
            #%%
            self.p2.set_operating_mode("currentposition")
            #%%
            self.p2.set_current(100)
            #%%
            #%%
            self.p2.set_goal_position(location)
            #%%
        except:
            print "Failed to run commands."


    #inner gripper related functions
    ######################################################
    ######################################################

    def inner_gripper_open_fully(self, current):
        try:
            #%%
            self.p1.set_operating_mode("currentposition")
            #%%
            self.p1.set_current(current)
            #%%
            #%%
            self.p1.set_goal_position(0)
            #%%
        except:
            print "Failed to run commands."

    def inner_gripper_close_fully(self, current):
        try:
            #%%
            self.p1.set_operating_mode("currentposition")
            #%%
            self.p1.set_current(current)
            #%%
            #%%
            self.p1.set_goal_position(5000)
            #%%
        except:
            print "Failed to run commands."

    def inner_gripper_open_slightly(self, current_position):
        try:
            #%%
            self.p1.set_operating_mode("currentposition")
            #%%
            self.p1.set_current(8)
            #current_position=self.p1.read_current_position()
            #%%
            #current_position = current_position-130
            self.p1.set_goal_position(current_position)
            #%%
        except:
            print "Failed to run commands."


    def inner_gripper_close_force(self):
        try:
            #%%
            self.p1.set_operating_mode("current")
            #%%
            self.p1.set_current(100)
            #%%
        except:
            print "Failed to run commands."


    def inner_gripper_close_new(self, goal_position):
        try:
            #%%
            self.p1.set_operating_mode("currentposition")
            #%%
            self.p1.set_current(110)#0.3Nm
            i1=self.p1.read_current_position()
            while i1<3072:
                self.p1.set_goal_position(i1)
                current1=int(self.p1.read_current())
                if current1>100:
                    break;
                i1=i1+8
            '''
            while i_avg>goal_position:
                i_avg=i_avg-1
                self.p2.set_goal_position(i_avg)
                p3.set_goal_position(i_avg)
                current2=self.p2.read_current()
                current3=p3.read_current()
                if current2>100 and current3>100:
                    break;
            '''
            #%%
        except:
            print "Failed to run commands."
    def inner_gripper_disable_torque(self):
        try:
            #%%
            self.p1.disable_torque()
            #%%
        except:
            print "Failed to run commands."
    ####linear motor
    '''def linear_motor_move_forward():
        try:
            #%%
            p4.set_operating_mode("current")
            #%%
            p4.set_positive_direction("cw")
            p4.set_current(100)
        except:
            print "Failed to run commands."'''
