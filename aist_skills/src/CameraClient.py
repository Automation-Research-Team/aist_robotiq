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
    def __init__(self, name, camera_type, camera_info_topic, image_topic):
        self._name               = name
        self._camera_type       = camera_type
        self._camera_info_topic = camera_info_topic
        self._image_topic       = image_topic

    @property
    def name(self):
        return self._name

    @property
    def camera_type(self):
        return self._camera_type

    @property
    def camera_info_topic(self):
        return self._camera_info_topic

    @property
    def image_topic(self):
        return self._image_topic

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
                                          "/" + name + "/depth_map")
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
                                              "/" + name + "/depth/points")
