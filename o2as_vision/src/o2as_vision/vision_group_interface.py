
import rospy
from geometry_msgs.msg import Pose
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
        rospy.loginfo("search object")
        success, response = self._cad_matching.search(self._pcloud_filename, self._image_filename, object_id)

        if not success:
            return

        # add detected objects to the planning scene
        if (response.search_result.result_num == 0):
            rospy.loginfo("no object found")
        else:
            rospy.logdebug("add detected object to planning scene")
            self.add_detected_object_to_planning_scene(response.search_result.detected_objects[0])

        rospy.logdebug("VisionGroupInterface.find_object() end")

    def add_detected_object_to_planning_scene(self, obj):
        rospy.logdebug("VisionGroupInterface.add_detected_object_to_planning_scene() begin")
        rospy.logdebug("pos = (%f, %f, %f)", obj.pos3D.x, obj.pos3D.y, obj.pos3D.z)

    	# convert position and orientation of detected object
        pose = Pose()
        pose.position = obj.pos3D
        pose.orientation.w = 1.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0

        # # convert size 
    	scale = [0.001, 0.001, 0.001] # [mm to meter]

        # add mesh to planning scene
        object_name = "05_MBRFA30-2-P6"
        #cad_filename = "package://o2as_parts_description/meshes/05_MBRFA30-2-P6.dae"
        cad_filename = "/root/catkin_ws/src/o2as_parts_description/meshes/05_MBRFA30-2-P6.dae"
        self._planning_scene.addMesh(name=object_name+"_mesh", pose=pose, filename=cad_filename, scale=scale)

        # # add box to planning scene (test)
        # a = 0.05
        # pose = obj.pos3D
        # self._planning_scene.addBox(name=object_name+"_box", x=pose.x, y=pose.y, z=pose.z, size_x=a, size_y=a, size_z=a)

        rospy.logdebug("VisionGroupInterface.add_detected_object_to_planning_scene() end")
