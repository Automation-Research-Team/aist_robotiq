#!/usr/bin/env python

import precision_gripper_class as pgc
import time
import rospy
from o2as_precision_gripper.srv import *

def my_callback(req):
    print req
    print "joshua"
    rospy.loginfo("The Service has been called")
    if req.open_outer_gripper_fully and not(req.close_outer_gripper_fully):
        pgc.outer_gripper_open_force()
        #print "lala"
    elif not(req.open_outer_gripper_fully) and (req.close_outer_gripper_fully):
        pgc.outer_gripper_close_force()
        #print "lala2"
    
    else:
        print('Again!!!!!!!!!!!!!!!!!!!!')

    #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    #req.response.success = True
    return True

# pg = pgc.PrecisionGripper()
if __name__ == "__main__":
    #initialise the class here
    rospy.init_node("precision_gripper_server")
    my_service = rospy.Service('precision_gripper_command', PrecisionGripperCommand, my_callback)
    rospy.loginfo("Service precision_gripper is ready")
    rospy.spin()
