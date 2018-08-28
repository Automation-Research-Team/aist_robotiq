import rospy
from util import *
from o2as_realsense_camera.srv import *

# service names
CONNECT_SERVICE = "connect"
SAVE_FRAME_FOR_CAD_MATCHING_SERVICE = "save_frame_for_cad_matching"
SAVE_CAMERA_INFO_SERVICE = "save_camera_info"

# the class send request to the camera server
class RealSenseCameraClient(object):
    def __init__(self, ns=""):
        self._connect = ros_service_proxy(ns+CONNECT_SERVICE, Connect)
        self._save_frame_for_cad_matching = ros_service_proxy(ns+SAVE_FRAME_FOR_CAD_MATCHING_SERVICE, SaveFrameForCadMatching)
        self._save_camera_info = ros_service_proxy(ns+SAVE_CAMERA_INFO_SERVICE, SaveCameraInfo)

    def connect(self):
        return self._connect()

    def save_frame_for_cad_matching(self, pcloud_filename, image_filename):
        return self._save_frame_for_cad_matching(pcloud_filename, image_filename)

    def save_camera_info(self, filename):
        return self._save_camera_info(filename)
