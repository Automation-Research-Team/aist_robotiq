import rospy
import dynamic_reconfigure.client

######################################################################
#  class CameraMultiplexerClient                                     #
######################################################################
class CameraMultiplexerClient(object):
    def __init__(self, name="camera_multiplexer"):
        self._camera_names = rospy.get_param(name + "/camera_names", [])
        self._dyn_reconf = dynamic_reconfigure.client.Client(name, timeout=5.0)

    @property
    def camera_names(self):
        return self._camera_names

    @property
    def active_camera(self):
        ret = self._dyn_reconf.get_configuration()
        return self._camera_names[ret["active_camera"]]

    @active_camera.setter
    def active_camera(self, camera_name):
        self._dyn_reconf.update_configuration(
            {"active_camera": self._camera_names.index(camera_name)})

######################################################################
#  class RealSenseMultiplexerClient                                  #
######################################################################
class RealSenseMultiplexerClient(CameraMultiplexerClient):

    class RealSenseCamera(object):
        def __init__(self, name):
            super(RealSenseCamera, self).__init__()
            self._dyn_reconf = dynamic_reconfigure.client.Client(name,
                                                                 timeout=5.0)
            self.laser_power = 16
            self._recent_laser_power = self.laser_power

        @property
        def laser_power(self):
            ret = self._dyn_sensor.get_configuration()
            return ret["laser_power"]

        @laser_power.setter
        def laser_power(self, value):
            self._dyn_sensor.update_configuration({"laser_power": value})

        def enalbe_laser(self, enabled):
            if enabled:
                self.laser_power = self._recent_laser_power
            else:
                self._recent_laser_power = self.laser_power
                self.laser_power = 0

        def continuous_shot(self, enabled):
            self._dyn_camera.update_configuration({"enable_streaming": enabled})
            rospy.sleep(0.2)
            return True

    def __init__(self, name="camera_multiplexer"):
        super(RealSenseMultiplexerClient, self).__init__(name)
        self._cameras = dict(zip(self.camera_names,
                                 [RealSenseCamera(camera_name)
                                  for camera_name in self.camera_names]))

    @active_camera.setter
    def active_camera(self, camera_name):
        self._cameras[self.active_camera].enable_laser(False)
        super(RealSenseMultiplexerClient, self).active_camera = camera_name
        self._cameras[self.active_camera].enable_laser(True)
