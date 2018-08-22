#!/usr/bin/env python

import rospy
from std_srvs.srv import *
from o2as_msgs.srv import *
from o2as_vision.vision_manager import VisionManager

LOG_LEVEL = log_level=rospy.DEBUG
#LOG_LEVEL = log_level=rospy.INFO

FIND_OBJECT_SERVICE = "find_object"

class VisionNode(object):
    def __init__(self):
        rospy.logdebug("VisionNode.__init__() begin")
        try:
            # params
            image_dir = rospy.get_param("~image_dir")
            camera_list = rospy.get_param("~camera_list")

            # prepare
            self.manager = VisionManager()
            for camera in camera_list:
                group = self.manager.add_group(camera)
                group.set_image_dir(image_dir)
            self.manager.prepare()

            # service
            rospy.logdebug("start service %s",FIND_OBJECT_SERVICE)
            self._find_object_server = rospy.Service(FIND_OBJECT_SERVICE, FindObject, self.find_object)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e)) 

        rospy.logdebug("VisionNode.__init__() success")

    def find_object(self, req):
        return self.manager.find_object(req.camera, req.object_id, req.expected_position, req.position_tolerance)

if __name__ == "__main__":
    rospy.init_node('o2as_vision', anonymous=True, log_level=LOG_LEVEL)
    node = VisionNode()
    rospy.spin()
