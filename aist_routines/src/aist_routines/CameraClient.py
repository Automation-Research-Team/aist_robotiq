import rospy
import dynamic_reconfigure.client
import std_srvs.srv

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

    def continuous_shot(self, enable):
        return True

    def trigger_frame(self):
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
        cs = "/{}".format(self.name)
        self._dyn_reconf = dynamic_reconfigure.client.Client(cs, timeout=None)
        self._trigger_frame = rospy.ServiceProxy(cs + "/trigger_frame",
                                                 std_srvs.srv.Trigger)

    def continuous_shot(self, enable):
        self._dyn_reconf.update_configuration({"trigger_mode" :
                                               0 if enable else 1})
        return True

    def trigger_frame(self):
        return self._trigger_frame().success

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

######################################################################
#  class MonocularCamera                                             #
######################################################################
class MonocularCamera(CameraClient):
    def __init__(self, name="IIDCCamera"):
        super(RealsenseCamera, self).__init__(
            name, "area",
            "/" + name + "/camera0/camera_info",
            "/" + name + "/camera0/image")
        self._dyn_reconf = dynamic_reconfigure.client.Client("/" + self.name,
                                                             timeout=None)

    def continuous_shot(self, enable):
        self._dyn_reconf.update_configuration({"continuous_shot" : enable})
        return True
