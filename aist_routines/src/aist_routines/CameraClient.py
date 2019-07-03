import rospy
import actionlib
import std_srvs.srv

import robotiq_msgs.msg

import o2as_msgs.msg
import o2as_msgs.srv

######################################################################
#  global functions                                                  #
######################################################################
def clamp(x, x_min, x_max):
    return min(max(x, x_min), x_max)

######################################################################
#  class CameraClient                                                 #
######################################################################
class CameraClient(object):
    def __init__(self, name, type, camera_info_topic, image_topic,
                 pointcloud_topic="", depth_topic="", normal_topic=""):
        self._name              = name
        self._type              = type
        self._camera_info_topic = camera_info_topic
        self._image_topic       = image_topic
        self._pointcloud_topic  = pointcloud_topic
        self._depth_topic       = depth_topic
        self._normal_topic      = normal_topic

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    @property
    def camera_info_topic(self):
        return self._camera_info_topic

    @property
    def image_topic(self):
        return self._image_topic

    @property
    def pointcloud_topic(self):
        return self._pointcooud_topic

    @property
    def depth_topic(self):
        return self._depth_topic

    @property
    def normal_topic(self):
        return self._normal_topic

    def start_acquisition(self):
        return True

    def stop_acquisition(self):
        return True


######################################################################
#  class PhoXiCamera                                                 #
######################################################################
class PhoXiCamera(CameraClient):
    def __init__(self, name="a_phoxi_m_camera"):
        super(PhoXiCamera, self).__init__(str(name),
                                          "depth",
                                          "/" + name + "/camera_info",
                                          "/" + name + "/texture",
                                          "/" + name + "/pointcloud",
                                          "/" + name + "/depth_map",
                                          "/" + name + "/normal_map")
        cs = "/{}/".format(self.name)
        self._start_acquisition = rospy.ServiceProxy(cs + "start_acquisition",
                                                     std_srvs.srv.Trigger)
        self._stop_acquisition  = rospy.ServiceProxy(cs + "stop_acquisition",
                                                     std_srvs.srv.Trigger)

    def start_acquisition(self):
        return self._start_acquisition().success

    def stop_acquisition(self):
        return self._stop_acquisition().success


######################################################################
#  class RealsenseCamera                                             #
######################################################################
class RealsenseCamera(CameraClient):
    def __init__(self, name="a_bot_camera"):
        super(RealsenseCamera, self).__init__(name,
                                              "depth",
                                              "/" + name + "/rgb/camera_info",
                                              "/" + name + "/rgb/image_raw",
                                              "/" + name + "/depth/points")
