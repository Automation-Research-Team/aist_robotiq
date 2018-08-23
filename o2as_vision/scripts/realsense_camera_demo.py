#!/usr/bin/env python

import rospy
from std_srvs.srv import *
from o2as_msgs.srv import *
from o2as_vision.util import *
from o2as_vision.vision_manager import VisionManager

MAX_TRIAL = 30
FIND_OBJECT_SERVICE = "find_object"

if __name__ == "__main__":
    rospy.init_node('o2as_vision_demo', anonymous=True, log_level=rospy.DEBUG)

    ask_object_id = True
    if rospy.has_param("~object_id"):
        object_id = rospy.get_param("~object_id")
        ask_object_id = False

    try:
        find_object = ros_service_proxy(FIND_OBJECT_SERVICE, FindObject)

        if ask_object_id:
            object_id = raw_input("Enter the number of the part to be performed: ")
        while not rospy.core.is_shutdown():
            rospy.loginfo("search object with id: " + object_id)
            j = 0
            while not rospy.core.is_shutdown():
                req = FindObjectRequest()
                req.object_id = object_id
                req.camera = "b_bot_camera"
                res = find_object(req.expected_position, req.position_tolerance, req.object_id, req.camera)
                if res.success:
                    rospy.loginfo("object found at: ")
                    rospy.loginfo(res.object_pose)
                    break
                else:
                    rospy.loginfo("no object found. retry")
                if ask_object_id:
                    j=j+1
                    if j > MAX_TRIAL:
                        break
            
            rospy.rostime.wallsleep(0.1)
            if ask_object_id:
                object_id = raw_input("the number of the part: ")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e)) 
