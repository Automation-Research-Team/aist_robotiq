#!/usr/bin/env python

import rospy
from util import *
from std_srvs.srv import *
from o2as_vision.srv import FindObject, FindObjectRequest
from o2as_vision.vision_manager import VisionManager

FIND_OBJECT_SERVICE = "find_object"

if __name__ == "__main__":
    rospy.init_node('o2as_vision_demo', anonymous=True, log_level=rospy.DEBUG)

    try:
        object_id = rospy.get_param("~object_id")
        find_object = ros_service_proxy(FIND_OBJECT_SERVICE, FindObject)

        while not rospy.core.is_shutdown():
            # find object
            req = FindObjectRequest()
            req.object_id = object_id
            req.camera = "b_bot_camera"
            find_object(req.expected_position, req.position_tolerance, req.object_id, req.camera)
            rospy.rostime.wallsleep(0.5)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e)) 
