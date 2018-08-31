import rospy
import rospkg
import tf
import geometry_msgs.msg
from o2as_msgs.srv import *
from vision_couplet import VisionCouplet

#LOG_LEVEL = log_level=rospy.DEBUG
LOG_LEVEL = log_level=rospy.INFO

class PartsInfo(object):
    def __init__(self, object_id, name, type, cad, description):
        self.object_id = object_id
        self.name = name
        self.type = type
        self.cad = cad
        self.description = description

class VisionManager(object):
    """
    This is a helper class for the o2as_vision_server. It owns 
    """
    def __init__(self):
        self._items = dict()
        self._vision_couplets = dict()
        parts_list = rospy.get_param("/parts_list")
        for item in parts_list:
            object_id = str(item['id'])
            info = PartsInfo(object_id=object_id, name=item['name'], type=item['type'], cad=item['cad'], description=item['description'])
            self._items[object_id] = info
            rospy.logdebug("parts id=%s, name=%s, type=%s, cad=%s, desc=%s", object_id, info.name, info.type, info.cad, info.description)
        
    def find_object(self, camera, object_id, expected_position, position_tolerance):
        rospy.logdebug("VisionManager.find_object() begin")
        pos = expected_position.pose.position
        rospy.logdebug("camera = %s", camera)
        rospy.logdebug("object_id = %s", object_id)
        rospy.logdebug("expected_position = (%f, %f, %f)", pos.x, pos.y, pos.z)
        rospy.logdebug("position_tolerance = %f", position_tolerance)

        res = FindObjectResponse()
        res.success = False
        couplet = self.get_couplet(camera)
        detected_object = couplet.find_object(object_id, expected_position, position_tolerance)
        if detected_object == None:
            rospy.loginfo("no object found")
        else:
            rospy.logdebug("add detected object to planning scene")

            # change Pose to PoseStamped 
            # (cad matching always returns pose in depth image frame)
            depth_image_frame = camera + "_depth_image_frame"
            p = geometry_msgs.msg.PoseStamped()
            p.header.frame_id = depth_image_frame
            p.pose = detected_object.pose
            res.object_pose = p
            res.success = True

            self.add_detected_object_to_planning_scene(detected_object.object_id, p, couplet)

        rospy.logdebug("VisionManager.find_object() end")
        return res

    def add_detected_object_to_planning_scene(self, object_id, pose, couplet):
        # get parts info
        parts_info = self._items[str(object_id)]
        if parts_info is None:
            rospy.logerr("parts info was not found")
            return res

        # add detected objects to the planning scene
        rospack = rospkg.RosPack()
        path = rospack.get_path('o2as_parts_description')
        name = parts_info.type+"_mesh"
        cad_filename = path + "/meshes/" + parts_info.cad
        scale = [0.001, 0.001, 0.001] # [mm to meter]
        couplet.add_detected_object_to_planning_scene(name=name, pose=pose, cad_filename=cad_filename, scale=scale)

    def add_couplet(self, name):
        self._vision_couplets[name] = VisionCouplet(name)
        return self._vision_couplets[name]

    def get_couplet(self, name):
        return self._vision_couplets[name]
