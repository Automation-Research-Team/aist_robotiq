import rospy
import rospkg
import tf
from util import *
import geometry_msgs.msg
from o2as_msgs.srv import *
from o2as_parts_description.srv import *
from vision_group_interface import VisionGroupInterface

GET_PARTS_INFO_SERVICE = "get_parts_info"

class VisionManager(object):
    def __init__(self):
        self._items = dict()
        self._get_parts_info = ros_service_proxy(GET_PARTS_INFO_SERVICE, GetPartsInfo)
    
    def get_parts_info(self, object_id):
        rospy.logdebug("VisionNode.get_parts_info() begin")
        rospy.logdebug("object_id = %s", object_id)
        try:
            response = self._get_parts_info(object_id)
            if response.exists:
                rospy.logdebug("VisionNode.get_parts_info() end")
                return response.parts_info
        except rospy.ServiceException as e:
            rospy.logerr("service call failed: %s", str(e)) 
        return None
        
    def prepare(self):
        for key in self._items:
            self._items[key].prepare()

    def find_object(self, camera, object_id, expected_position, position_tolerance):
        rospy.logdebug("VisionManager.find_object() begin")
        pos = expected_position.pose.position
        rospy.logdebug("camera = %s", camera)
        rospy.logdebug("object_id = %s", object_id)
        rospy.logdebug("expected_position = (%f, %f, %f)", pos.x, pos.y, pos.z)
        rospy.logdebug("position_tolerance = %f", position_tolerance)

        res = FindObjectResponse()
        res.success = False
        group = self.get_group(camera)
        detected_object = group.find_object(object_id, expected_position, position_tolerance)
        if detected_object == None:
            rospy.loginfo("no object found")
        else:
            rospy.logdebug("add detected object to planning scene")
            res.success = True

            q = tf.transformations.quaternion_from_euler(detected_object.rot3D.x, detected_object.rot3D.y, detected_object.rot3D.z)
            p = geometry_msgs.msg.PoseStamped()
            p.header.frame_id = camera + "_depth_frame"
            p.pose.position = detected_object.pos3D
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            res.object_pose = p

            self.add_detected_object_to_planning_scene(detected_object, group)
        rospy.logdebug("VisionManager.find_object() end")
        return res

    def add_detected_object_to_planning_scene(self, obj, group):
        # get parts info
        parts_info = self.get_parts_info(str(obj.object_id))
        if parts_info is None:
            rospy.logerr("parts info was not found")
            return res

        # add detected objects to the planning scene
        rospack = rospkg.RosPack()
        path = rospack.get_path('o2as_parts_description')
        object_name = parts_info.type
        cad_filename = path + "/meshes/" + parts_info.cad

        # convert position and orientation of detected object
        q = tf.transformations.quaternion_from_euler(obj.rot3D.x, obj.rot3D.y, obj.rot3D.z)
        pose = geometry_msgs.msg.Pose()
        pose.position = obj.pos3D
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        # convert size 
        scale = [0.001, 0.001, 0.001] # [mm to meter]
        
        rospy.logdebug("name = %s", object_name)
        rospy.logdebug("cad_filename = %s", cad_filename)
        rospy.logdebug("pos = (%f, %f, %f)", pose.position.x, pose.position.y, pose.position.z)
        rospy.logdebug("rot = (%f, %f, %f)", obj.rot3D.x, obj.rot3D.y, obj.rot3D.z)
        rospy.logdebug("scale = (%f, %f, %f)", scale[0], scale[1], scale[2])
        group.add_detected_object_to_planning_scene(name=object_name+"_mesh", pose=pose, cad_filename=cad_filename, scale=scale)

    def add_group(self, name):
        self._items[name] = VisionGroupInterface(name)
        return self._items[name]

    def get_group(self, name):
        return self._items[name]
