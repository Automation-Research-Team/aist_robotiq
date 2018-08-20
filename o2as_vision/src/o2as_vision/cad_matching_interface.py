import rospy
from util import *
from o2as_cad_matching_msgs.srv import *
from o2as_cad_matching_msgs.msg import *

# service name
LOAD_MODEL_DATA_SERVICE = "load_model_data"
PREPARE_SERVICE = "prepare"
SEARCH_SERVICE = "search"

class CadMatchingInterface(object):
    def __init__(self, group_name=""):
        rospy.logdebug("CadMatchingInterface.__init__() begin")
        rospy.logdebug("group_name = %s", group_name)
        self._group_name = group_name
        self._load_model_data = ros_service_proxy(group_name+"/"+LOAD_MODEL_DATA_SERVICE, LoadModelData)
        self._prepare = ros_service_proxy(group_name+"/"+PREPARE_SERVICE, Prepare)
        self._search = ros_service_proxy(group_name+"/"+SEARCH_SERVICE, Search)
        rospy.logdebug("CadMatchingInterface.__init()__ end")

    # load trained model data for cad matching
    def load_model_data(self, model_filename=""):
        rospy.logdebug("CadMatchingInterface.load_model_data() begin")
        try:
            self._load_model_data(model_filename)
        except rospy.ServiceException as e:
            rospy.logerr("service call failed: %s", str(e)) 
            return False
        rospy.logdebug("CadMatchingInterface.load_model_data() end")
        return True

    # prepare for cad matching
    def prepare(self):
        rospy.logdebug("CadMatchingInterface.repare() begin")
        try:
            self._prepare()
        except rospy.ServiceException as e:
            rospy.logerr("service call failed: %s", str(e)) 
            return False
        rospy.logdebug("CadMatchingInterface.prepare() end")
        return True

    # search object
    def search(self, pcloud_filename, image_filename="", object_id="0"):
        rospy.logdebug("CadMatchingInterface.search() begin")
        try:
            response = self._search(pcloud_filename, image_filename, object_id)
        except rospy.ServiceException as e:
            rospy.logerr("service call failed: %s", str(e)) 
            return False, None
        rospy.logdebug("CadMatchingInterface.search() end")
        return True, response
