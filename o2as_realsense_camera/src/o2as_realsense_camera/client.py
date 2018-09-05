import rospy
from o2as_realsense_camera.srv import *

def ros_service_proxy(service_name, service_type):
    proxy = None
    try:
        rospy.logdebug("wait for service %s", service_name)
        rospy.wait_for_service(service_name)
        proxy = rospy.ServiceProxy(service_name, service_type)
    except rospy.ServiceException as e:
        rospy.logerr("service error: %s", str(e)) 
    return proxy
    
# the class send request to the camera server
class RealSenseCameraClient(object):
    def __init__(self, server_node_name=""):
        ns = "/"+server_node_name+"/"
        self._get_frame = ros_service_proxy(ns+"get_frame", GetFrame)
        self._dump_frame = ros_service_proxy(ns+"dump_frame", DumpFrame)

    def get_frame(self, dump=False, publish=False):
        return self._get_frame(dump, publish)

    def dump_frame(self, path, color_image_filename, depth_image_filename, point_cloud_filename):
        return self._dump_frame(path, color_image_filename, depth_image_filename, point_cloud_filename)
