######################################################################
#  class FTSensorClient                                              #
######################################################################
class FTSensorClient(object):
    def __init__(self, name, type, wrench_topic=""):
        self._name         = name
        self._type         = type
        self._wrench_topic = wrench_topic

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
#  class FT300                                                       #
######################################################################
class FT300(FTSensorClient):
    def __init__(self, name="a_phoxi_m_camera"):
