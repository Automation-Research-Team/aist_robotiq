
import rospy
import tf
import numpy as np
from numpy import linalg as LA
from o2as_realsense_camera.client import RealSenseCameraClient
from o2as_cad_matching.cad_matching_client import CadMatchingClient
import moveit_commander

class VisionCouplet(object):
    def __init__(self, couplet_name=""):
        rospy.logdebug("VisionCouplet.__init__() begin")
        rospy.logdebug("couplet_name = %s", couplet_name)

        # camera and cad matching
        self._couplet_name = couplet_name
        self._camera = RealSenseCameraClient("/"+couplet_name+"/")
        self._cad_matching = CadMatchingClient("/"+couplet_name+"/")

        # planning scene
        #self._camera_frame = "/" + self._couplet_name + "_depth_frame"
        self._camera_frame = "/" + self._couplet_name + "_depth_image_frame"
        self._planning_scene = moveit_commander.PlanningSceneInterface()

        rospy.logdebug("VisionCouplet.__init__() end")

    def set_image_dir(self, image_dir):
        rospy.logdebug("VisionCouplet.set_image_dir() begin")

        # point cloud file and image file is saved into the image_dir.
        self._image_dir = image_dir
        self._pcloud_filename = self._image_dir + "/" + self._couplet_name + ".dat"
        self._image_filename = self._image_dir + "/" + self._couplet_name + ".png"

        rospy.logdebug("VisionCouplet.set_image_dir() end")

    def prepare(self):
        rospy.logdebug("VisionCouplet.prepare() begin")

        # connect to the camera
        if not self._camera.connect():
    		rospy.logerr("camera connect failed")

        rospy.logdebug("VisionCouplet.prepare() end")

    def find_objects(self, object_id):
        rospy.logdebug("VisionCouplet.find_objects() begin")

        # get image from sensor
        rospy.logdebug("save frame for cad matching")
        self._camera.save_frame_for_cad_matching(self._pcloud_filename, self._image_filename)

        # cad matching
        rospy.logdebug("search object")
        response = self._cad_matching.search(self._pcloud_filename, self._image_filename, object_id)

        rospy.logdebug("VisionCouplet.find_objects() end")
        if not response.success:
            return []
        return response.search_result.detected_objects

    def find_object(self, object_id, expected_position, position_tolerance):
        rospy.logdebug("VisionCouplet.find_object() begin")
        detected_objects = self.find_objects(object_id)

        n = len(detected_objects) 
        if n > 0:
            # choose nearest object from expected position within tolerance
            # expected position should be specified with depth image frame of the camera
            rospy.logdebug("%d objects found. Selecting nearest.", n)
            ix_closest = 0
            tolerance = float("inf")
            for i in range(n):
                obj = detected_objects[i]
                expected_pos = np.array([expected_position.pose.position.x, expected_position.pose.position.y, expected_position.pose.position.z])
                seen_pos = np.array([obj.pose.position.x, obj.pose.position.y, obj.pose.position.z])
                dist = LA.norm(expected_pos-seen_pos)
                if dist < tolerance:
                    tolerance = dist
                    ix_closest = i
                rospy.logdebug("expected=({},{},{}), seen=({},{},{}), d={}".format(expected_pos[0], expected_pos[1], expected_pos[2], seen_pos[0], seen_pos[1], seen_pos[2], dist))
            return detected_objects[ix_closest]
        else:
            return None

    def add_detected_object_to_planning_scene(self, name, pose, cad_filename, scale):
        rospy.logdebug("VisionCouplet.add_detected_object_to_planning_scene() begin")
        rospy.logdebug("name: " + name)
        rospy.logdebug("cad_filename: " + cad_filename)
        rospy.logdebug("pose:")
        rospy.logdebug(pose)
        rospy.logdebug("scale: (%f, %f, %f)", scale[0], scale[1], scale[2])

        self._planning_scene.add_mesh(name=name, pose=pose, filename=cad_filename, size=scale)
        #self._planning_scene.add_box(name=name+"_box", pose=pose, size=(0.2, 0.2, 0.2)) # debug
        rospy.logdebug("VisionCouplet.add_detected_object_to_planning_scene() end")
