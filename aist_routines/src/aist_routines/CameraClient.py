import rospy
import dynamic_reconfigure.client
import std_srvs.srv

######################################################################
#  class CameraClient                                                #
######################################################################
class CameraClient(object):
    def __init__(self, name, camera_type):
        self._name = name
        self._type = camera_type

    @staticmethod
    def create(name, type_name):
        ClientClass = globals()[type_name]
        if rospy.get_param("use_real_robot", False):
            return ClientClass(name)
        else:
            return ClientClass.base(name)

    @property
    def name(self):
        return self._name

    @property
    def type(self):
        return self._type

    def continuous_shot(self, enabled):
        return True

    def trigger_frame(self):
        return True

######################################################################
#  class MonocularCamera                                             #
######################################################################
class MonocularCamera(CameraClient):
    def __init__(self, name="IIDCCamera"):
        super(MonocularCamera, self).__init__(name, "area")
        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=None)

    @staticmethod
    def base(name):
        return CameraClient(name, "area")

    def continuous_shot(self, enabled):
        self._dyn_reconf.update_configuration({"continuous_shot" : enabled})
        return True

######################################################################
#  class DepthCamera                                                 #
######################################################################
class DepthCamera(CameraClient):
    def __init__(self, name):
        super(DepthCamera, self).__init__(name, "depth")

    @staticmethod
    def base(name):
        return CameraClient(name, "depth")

######################################################################
#  class RealSenseCamera                                             #
######################################################################
class RealSenseCamera(DepthCamera):
    def __init__(self, name="a_bot_camera"):
        super(RealSenseCamera, self).__init__(name)
        self._dyn_camera = dynamic_reconfigure.client.Client(name, timeout=5.0)
        self._dyn_sensor = dynamic_reconfigure.client.Client(
                               name + "/coded_light_depth_sensor", timeout=5.0)
        self._recent_laser_power = 16
        self.laser_power = 0

    @property
    def laser_power(self):
        ret = self._dyn_sensor.get_configuration()
        return ret["laser_power"]

    @laser_power.setter
    def laser_power(self, value):
        if value != 0:
            self._recent_laser_power = value
        self._dyn_sensor.update_configuration({"laser_power" : value})

    def continuous_shot(self, enabled):
        if enabled:
            self.laser_power = self._recent_laser_power
        else:
            self.laser_power = 0
        self._dyn_camera.update_configuration({"enable_streaming" : enabled})
        rospy.sleep(0.2)
        return True

######################################################################
#  class PhoXiCamera                                                 #
######################################################################
class PhoXiCamera(DepthCamera):
    def __init__(self, name="a_phoxi_m_camera"):
        super(PhoXiCamera, self).__init__(name)
        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=5.0)
        self._trigger_frame = rospy.ServiceProxy(name + "/trigger_frame",
                                                 std_srvs.srv.Trigger)

    def continuous_shot(self, enabled):
        self._dyn_reconf.update_configuration({"trigger_mode" :
                                               0 if enabled else 1})
        return True

    def trigger_frame(self):
        return self._trigger_frame().success
