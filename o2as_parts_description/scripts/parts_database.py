#!/usr/bin/env python

import rospy
from std_srvs.srv import *
from o2as_parts_description.msg import PartsInfo
from o2as_parts_description.srv import GetPartsInfo, GetPartsInfoResponse

#LOG_LEVEL = log_level=rospy.DEBUG
LOG_LEVEL = log_level=rospy.INFO

GET_PARTS_INFO_SERVICE = "get_parts_info"

class PartsDatabase(object):
    def __init__(self):
        rospy.logdebug("PartsDatabase.__init__() begin")
        try:
            self.init_parts_dict()
            rospy.logdebug("start service %s",GET_PARTS_INFO_SERVICE)
            self._get_parts_info_server = rospy.Service(GET_PARTS_INFO_SERVICE, GetPartsInfo, self.get_parts_info)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", str(e)) 
        rospy.logdebug("PartsDatabase.__init__() success")

    def init_parts_dict(self):
        self._parts_dict = dict()
        parts_list = rospy.get_param("~parts_list")
        for item in parts_list:
            object_id = str(item['id'])
            info = PartsInfo(object_id=object_id, name=item['name'], type=item['type'], cad=item['cad'], description=item['description'])
            self._parts_dict[object_id] = info
            rospy.logdebug("parts id=%s, name=%s, type=%s, cad=%s, desc=%s", object_id, info.name, info.type, info.cad, info.description)

    def get_parts_info(self, req):
        rospy.logdebug("PartsDatabase.get_parts_info() begin")
        rospy.logdebug("object_id = %s", req.object_id)
        res = GetPartsInfoResponse()
        if req.object_id in self._parts_dict.keys():
            rospy.logdebug("parts_info found")
            res.exists = True
            res.parts_info = self._parts_dict[req.object_id]
        else:
            rospy.logdebug("parts_info not found")
            res.exists = False
        rospy.logdebug("PartsDatabase.get_parts_info() end")
        return res

if __name__ == "__main__":
    rospy.init_node('o2as_parts_database', anonymous=True, log_level=LOG_LEVEL)
    node = PartsDatabase()
    rospy.spin()
