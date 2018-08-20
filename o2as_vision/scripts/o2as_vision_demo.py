#!/usr/bin/env python

import rospy
from std_srvs.srv import *
from o2as_vision.srv import FindObjects, FindObjectsRequest
from o2as_vision.vision_manager import VisionManager

FIND_OBJECTS_SERVICE = "find_objects"

if __name__ == "__main__":
    rospy.init_node('o2as_vision_demo', anonymous=True, log_level=rospy.DEBUG)

    try:
        rospy.logdebug("wait for service %s", FIND_OBJECTS_SERVICE)
        rospy.wait_for_service(FIND_OBJECTS_SERVICE)
        find_objects = rospy.ServiceProxy(FIND_OBJECTS_SERVICE, FindObjects)

        while not rospy.core.is_shutdown():
            req = FindObjectsRequest()
                        
            #  7 : bearing housing
            # 11 : output pulley
            # 13 : idler pulley
            # -1 : all
            req.object_id = "-1"
            req.camera = "b_bot_camera"
            find_objects(req.expected_position, req.position_tolerance, req.object_id, req.camera)
            rospy.rostime.wallsleep(0.5)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e)) 
