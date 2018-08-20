import rospy
from util import *
from o2as_realsense_camera.srv import *

# service name
CONNECT_SERVICE = "connect"
SAVE_FRAME_FOR_CAD_MATCHING_SERVICE = "save_frame_for_cad_matching"

class RealSenseCameraInterface(object):
    def __init__(self, group_name=""):
        rospy.logdebug("RealSenseCameraInterface.__init__() begin")
        rospy.logdebug("group_name = %s", group_name)
        self._group_name = group_name
        self._connect = ros_service_proxy(group_name+"/"+CONNECT_SERVICE, Connect)
        self._save_frame_for_cad_matching = ros_service_proxy(group_name+"/"+SAVE_FRAME_FOR_CAD_MATCHING_SERVICE, SaveFrameForCadMatching)
        rospy.logdebug("RealSenseCameraInterface.__init__() end")

    # connect to the camera
    def connect(self):
        rospy.logdebug("RealSenseCameraInterface.connect() begin")
        try:
            self._connect()
        except rospy.ServiceException as e:
            rospy.logerr("service call failed: %s", str(e)) 
            return False
        rospy.logdebug("RealSenseCameraInterface.connect() end")
        return True

    # save frame for cad matching
    def save_frame_for_cad_matching(self, pcloud_filename, image_filename):
        rospy.logdebug("RealSenseCameraInterface.save_frame_for_cad_matching() begin")
        rospy.logdebug("pcloud_filename = %s", pcloud_filename)
        rospy.logdebug("image_filename = %s", image_filename)
        try:
            self._save_frame_for_cad_matching(pcloud_filename, image_filename)
        except rospy.ServiceException as e:
            rospy.logerr("service call failed: %s", str(e)) 
            return False
        rospy.logdebug("RealSenseCameraInterface.save_frame_for_cad_matching() end")
        return True
