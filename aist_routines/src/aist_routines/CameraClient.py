import rospy
import dynamic_reconfigure.client
import std_srvs.srv

######################################################################
#  class CameraClient                                                #
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

    @staticmethod
    def create(type_name, kwargs):
        ClientClass = globals()[type_name]
        if rospy.get_param("use_real_robot", False):
            return ClientClass(**kwargs)
        else:
            return ClientClass.base(**kwargs)

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
        super(PhoXiCamera, self).__init__(*PhoXiCamera._initargs(name))
        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=5.0)
        self._trigger_frame = rospy.ServiceProxy(name + "/trigger_frame",
                                                 std_srvs.srv.Trigger)

    @staticmethod
    def base(name):
        return CameraClient(*PhoXiCamera._initargs(name))

    @staticmethod
    def _initargs(name):
        return (name, "depth",       name + "/camera_info",
                name + "/texture",   name + "/pointcloud",
                name + "/depth_map", name + "/normal_map")

    def continuous_shot(self, enable):
        self._dyn_reconf.update_configuration({"trigger_mode" :
                                               0 if enable else 1})
        return True

    def trigger_frame(self):
        return self._trigger_frame().success

######################################################################
#  class DepthCamera                                                 #
######################################################################
class DepthCamera(CameraClient):
    def __init__(self, name="a_bot_camera",
                 camera_info_topic="color/camera_info",
                 image_topic="color/image_raw",
                 pointcloud_topic="depth/color/points",
                 depth_topic="aligned_depth_to_color/image_raw"):
        super(DepthCamera, self).__init__(*DepthCamera._initargs(
            name, camera_info_topic,
            image_topic, pointcloud_topic, depth_topic))
#        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=5.0)

    @staticmethod
    def base(name,
             camera_info_topic, image_topic, pointcloud_topic, depth_topic):
        return CameraClient(*DepthCamera._initargs(name,
                                                   camera_info_topic,
                                                   image_topic,
                                                   pointcloud_topic,
                                                   depth_topic))

    @staticmethod
    def _initargs(name, camera_info_topic,
                  image_topic, pointcloud_topic, depth_topic):
        return (name, "depth",
                name + "/" + camera_info_topic,
                name + "/" + image_topic,
                name + "/" + pointcloud_topic,
                name + "/" + depth_topic)

 #   def continuous_shot(self, enable):
 #       self._dyn_reconf.update_configuration({"emitter_enabled" : enable})
 #       return True

######################################################################
#  class MonocularCamera                                             #
######################################################################
class MonocularCamera(CameraClient):
    def __init__(self, name="IIDCCamera"):
        super(RealsenseCamera, self).__init__(*MonocularCamera._initargs(name))
        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=None)

    @staticmethod
    def base(name):
        return CameraClient(*MonocularCamera._initargs(name))

    @staticmethod
    def _initargs(name):
        return (name, "area",
                name + "/camera0/camera_info", name + "/camera0/image")

    def continuous_shot(self, enable):
        self._dyn_reconf.update_configuration({"continuous_shot" : enable})
        return True
