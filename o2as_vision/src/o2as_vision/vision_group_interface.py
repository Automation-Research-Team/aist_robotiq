
import rospy
from realsense_camera_interface import RealSenseCameraInterface
from cad_matching_interface import CadMatchingInterface
from planning_scene_interface import PlanningSceneInterface

class VisionGroupInterface(object):
    def __init__(self, group_name=""):
        rospy.logdebug("VisionGroupInterface.__init__() begin")
        rospy.logdebug("group_name = %s", group_name)

        # camera and cad matching
        self._group_name = group_name
        self._camera = RealSenseCameraInterface(group_name) 
        self._cad_matching = CadMatchingInterface(group_name)

        # planning scene
        self._camera_frame = "/" + self._group_name + "_depth_frame"
        self._planning_scene = PlanningSceneInterface(self._camera_frame)

        rospy.logdebug("VisionGroupInterface.__init__() end")

    def set_image_dir(self, image_dir):
        rospy.logdebug("VisionGroupInterface.set_image_dir() begin")

        # point cloud file and image file is saved into the image_dir.
        self._image_dir = image_dir
        self._pcloud_filename = self._image_dir + "/" + self._group_name + ".dat"
        self._image_filename = self._image_dir + "/" + self._group_name + ".png"

        rospy.logdebug("VisionGroupInterface.set_image_dir() end")

    def prepare(self):
        rospy.logdebug("VisionGroupInterface.prepare() begin")

        # load model data for cad matching
        if not self._cad_matching.load_model_data():
    		rospy.logerr("cad matching load model data failed")

        # prepare for cad matching
        if not self._cad_matching.prepare():
    		rospy.logerr("cad matching prepare failed")

        # connect to the camera
        if not self._camera.connect():
    		rospy.logerr("camera connect failed")

        rospy.logdebug("VisionGroupInterface.prepare() end")

    def find_object(self, object_id):
        rospy.logdebug("VisionGroupInterface.find_object() begin")

        # get image from sensor
        rospy.logdebug("save frame for cad matching")
        self._camera.save_frame_for_cad_matching(self._pcloud_filename, self._image_filename)

        # cad matching
        rospy.logdebug("search object")
        success, response = self._cad_matching.search(self._pcloud_filename, self._image_filename, object_id)

        rospy.logdebug("VisionGroupInterface.find_object() end")
        if not success:
            return []
        return response.search_result.detected_objects

    def add_detected_object_to_planning_scene(self, name, pose, cad_filename, scale):
        rospy.logdebug("VisionGroupInterface.add_detected_object_to_planning_scene() begin")
        self._planning_scene.addMesh(name=name, pose=pose, filename=cad_filename, scale=scale)
        # # add box to planning scene (test)
        # a = 0.05
        # self._planning_scene.addBox(name=object_name+"_box", x=pose.position.x, y=pose.position.y, z=pose.position.z, size_x=a, size_y=a, size_z=a)
        rospy.logdebug("VisionGroupInterface.add_detected_object_to_planning_scene() end")
