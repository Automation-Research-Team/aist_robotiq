import rospy
import dynamic_reconfigure.client
from std_srvs import srv as ssrv

#########################################################################
#  class DepthFilterClient                                              #
#########################################################################
class DepthFilterClient(object):
    def __init__(self, name="depth_filter"):
        super(DepthFilterClient, self).__init__()
        self._saveBG     = rospy.ServiceProxy(name + "/saveBG",  ssrv.Trigger)
        self._capture    = rospy.ServiceProxy(name + "/capture", ssrv.Trigger)
        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=5.0)

    def saveBG(self):
        return self._saveBG().success

    def capture(self):
        return self._capture().success

    @property
    def background_threshold(self):
        ret = self._dyn_reconf.get_configuration()
        return ret["thresh_bg"]

    @background_threshold.setter
    def background_threshold(self, value):
        self._dyn_reconf.update_configuration({"thresh_bg": value})

    @property
    def depth_range(self):
        ret = self._dyn_reconf.get_configuration()
        return (ret["near"], ret["far"])

    @depth_range.setter
    def depth_range(self, range):
        self._dyn_reconf.update_configuration({"near": range[0],
                                               "far":  range[1]})

    @property
    def roi(self):
        ret = self._dyn_reconf.get_configuration()
        return (ret["left"], ret["top"], ret["right"], ret["bottom"])

    @roi.setter
    def roi(self, bbox):
        self._dyn_reconf.update_configuration({"left"  : bbox[0],
                                               "top"   : bbox[1],
                                               "right" : bbox[2],
                                               "bottom": bbox[3]})

    @property
    def window_radius(self):
        ret = self._dyn_reconf.get_configuration()
        return ret["window_radius"]

    @window_radius.setter
    def window_radius(self, value):
        self._dyn_reconf.update_configuration({"window_radius": value})

    @property
    def scale(self):
        ret = self._dyn_reconf.get_configuration()
        return ret["scale"]

    @scale.setter
    def scale(self, value):
        self._dyn_reconf.update_configuration({"scale": value})
