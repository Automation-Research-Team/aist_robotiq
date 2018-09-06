import rospy
from o2as_realsense_camera.client import RealSenseCameraClient

class RealSenseCamera(object):
    def __init__(self, name=""):
        self._name = name
        self._client = RealSenseCameraClient(name)
        self._namespace = "/"+name+"/"

    def start(self):
        return True

    def get_frame(self):
        resp = self._client.get_frame()
        return resp.point_cloud, resp.color_image

    def get_depth_image_frame(self):
        return self._name + "_depth_image_frame"
