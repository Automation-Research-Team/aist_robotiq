#!/usr/bin/env python

import rospy
from std_srvs.srv import *
from o2as_vision.srv import FindObjects, FindObjectsResponse
from o2as_vision.vision_manager import VisionManager

FIND_OBJECTS_SERVICE = "find_objects"

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
            rospy.logdebug("start service %s",FIND_OBJECTS_SERVICE)
            self._find_objects_server = rospy.Service(FIND_OBJECTS_SERVICE, FindObjects, self.find_objects)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e)) 

        rospy.logdebug("VisionNode.__init__() success")

    def find_objects(self, req):
        res = FindObjectsResponse()

        rospy.logdebug("VisionNode.find_objects() begin")
        pos = req.expected_position.pose.position
        rospy.logdebug("expected_position = (%f, %f, %f)", pos.x, pos.y, pos.z)
        rospy.logdebug("position_tolerance = %f", req.position_tolerance)
        rospy.logdebug("object_id = %s", req.object_id)
        rospy.logdebug("camera = %s", req.camera)

        #self.manager.update_scene() # test
        group = self.manager.get_group(req.camera)
        group.find_object(req.object_id)

        ###############################
        # TODO: implement here
        ###############################

        rospy.logdebug("VisionNode.find_objects() begin")
        return res

if __name__ == "__main__":
    rospy.init_node('o2as_vision', anonymous=True, log_level=rospy.DEBUG)
    node = VisionNode()
    rospy.spin()
