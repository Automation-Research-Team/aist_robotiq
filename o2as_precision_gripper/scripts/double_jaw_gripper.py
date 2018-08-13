import lib_robotis_xm430 as xm430
import sys
import time

pitch=1.98#9./5.*1.05*1.02#found from experiment
#%%
#joshua
#20180620
dynamixel1 = xm430.USB2Dynamixel_Device( '/dev/ttyUSB0', baudrate = 57600 )
#xm.find_servos(dyn)
#%%
p1 = xm430.Robotis_Servo2( dynamixel1, 1, series = "XM" )  #inner gripper
#%%
p2 = xm430.Robotis_Servo2( dynamixel1, 2, series = "XM" )  #outer gripper
#%%

#%%
#outer gripper related functions
def outer_gripper_close_new(goal_position):
    try:
        #%%
        p2.set_operating_mode("currentposition")
        #%%
        p2.set_current(100) #0.3Nm
        i2 = p2.read_current_position()
        i_avg = int(i2)
        while i_avg < goal_position:
            p2.set_goal_position(i_avg)
            current2 = int(p2.read_current())
            if current2 > 100:
                break
            i_avg = i_avg + 8
        #%%
    except:
        print "Failed to run commands."


def inner_innergripper_read_current_position():
    try:
        x=p1.read_current_position()
        print"id1="+str(x)
    except:
        print "Failed to run commands."

def outer_gripper_read_current_position():
    try:
        #%%
        x=p2.read_current_position()
        print "id2="+str(x)
    except:
        print "Failed to run commands."


def outer_gripper_open_fully():
    try:
        #%%
        p2.set_operating_mode("currentposition")
        #%%
        p2.set_current(30)
        #%%
        #%%
        p2.set_goal_position(-30000)
        #%%
    except:
        print "Failed to run commands."


def outer_gripper_close_fully():
    try:
        #%%
        p2.set_operating_mode("currentposition")
        #%%
        p2.set_current(20)
        #%%
        #%%
        p2.set_goal_position(70240)
        #%%
    except:
        print "Failed to run commands."

def outer_gripper_disable_torque():
    try:
        #%%
        p2.disable_torque()
        #%%
    except:
        print "Failed to run commands."


def outer_gripper_open_to(location):#still editing
    try:
        #%%
        p2.set_operating_mode("currentposition")
        #%%
        p2.set_current(100)
        #%%
        #%%
        p2.set_goal_position(location)
        #%%
    except:
        print "Failed to run commands."


#inner gripper related functions
######################################################
######################################################

def inner_gripper_open_fully(current):
    try:
        #%%
        p1.set_operating_mode("currentposition")
        #%%
        p1.set_current(current)
        #%%
        #%%
        p1.set_goal_position(0)
        #%%
    except:
        print "Failed to run commands."

def inner_gripper_close_fully(current):
    try:
        #%%
        p1.set_operating_mode("currentposition")
        #%%
        p1.set_current(current)
        #%%
        #%%
        p1.set_goal_position(5000)
        #%%
    except:
        print "Failed to run commands."

def inner_gripper_open_slightly(current_position):
    try:
        #%%
        p1.set_operating_mode("currentposition")
        #%%
        p1.set_current(8)
        #current_position=p1.read_current_position()
        #%%
        #current_position = current_position-130
        p1.set_goal_position(current_position)
        #%%
    except:
        print "Failed to run commands."


def inner_gripper_close_force():
    try:
        #%%
        p1.set_operating_mode("current")
        #%%
        p1.set_current(100)
        #%%
    except:
        print "Failed to run commands."


def inner_gripper_close_new(goal_position):
    try:
        #%%
        p1.set_operating_mode("currentposition")
        #%%
        p1.set_current(110)#0.3Nm
        i1=p1.read_current_position()
        while i1<3072:
            p1.set_goal_position(i1)
            current1=int(p1.read_current())
            if current1>100:
                break;
            i1=i1+8
        '''
        while i_avg>goal_position:
            i_avg=i_avg-1
            p2.set_goal_position(i_avg)
            p3.set_goal_position(i_avg)
            current2=p2.read_current()
            current3=p3.read_current()
            if current2>100 and current3>100:
                break;
        '''
        #%%
    except:
        print "Failed to run commands."
def inner_gripper_disable_torque():
    try:
        #%%
        p1.disable_torque()
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
