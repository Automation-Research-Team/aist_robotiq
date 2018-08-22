import rospy
from std_srvs.srv import *
from o2as_msgs.srv import *
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point

FIND_OBJECT_SERVICE = "find_object"

def ros_service_proxy(service_name, service_type):
    proxy = None
    try:
        rospy.logdebug("wait for service %s", service_name)
        rospy.wait_for_service(service_name)
        proxy = rospy.ServiceProxy(service_name, service_type)
    except rospy.ServiceException as e:
        rospy.logerr("service error: " + str(e))
    return proxy

class VisionProxy(object):
    def __init__(self):
        self._find_object = ros_service_proxy(FIND_OBJECT_SERVICE, FindObject)

    def find_object(self, expected_position, position_tolerance, object_id, camera):
        rospy.loginfo("SkillProxy.find_object() begin")
        rospy.loginfo("expected_position: ")
        rospy.loginfo(expected_position.pose.position)
        rospy.loginfo("position_tolerance = " + str(position_tolerance))
        rospy.loginfo("object_id = " + str(object_id))
        rospy.loginfo("camera = " + str(camera))
        try:
            res = self._find_object(expected_position, position_tolerance, object_id, camera)
            if res.success:
                return res.object_pose
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))
        return None
