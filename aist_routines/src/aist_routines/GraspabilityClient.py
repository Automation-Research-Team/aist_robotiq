import sys, os, collections
import rospy, rospkg, rosparam
import actionlib
import numpy as np
import cv2
import std_msgs
import base
from math              import radians, cos, sin, sqrt, pi
from sensor_msgs       import msg as smsg
from geometry_msgs     import msg as gmsg
from aist_graspability import msg as amsg
from aist_graspability import srv as asrv
from cv_bridge         import CvBridge, CvBridgeError
from tf                import TransformListener, transformations as tfs

######################################################################
#  class GraspabilityClient                                          #
######################################################################
class GraspabilityClient(object):

    def __init__(self, reference_frame):
        super(GraspabilityClient, self).__init__()

        rospack = rospkg.RosPack()
        d = rosparam.load_file(rospack.get_path("aist_routines") +
                               "/config/graspability_parameters.yaml")[0][0]
        self._params = base.paramtuples(d["graspability_parameters"])
        self._createBackgroundImage \
            = rospy.ServiceProxy("/aist_graspability/create_background_image",
                                 asrv.createBackgroundImage)
        self._createMaskImage \
            = rospy.ServiceProxy("/aist_graspability/create_mask_image", asrv.createMaskImage)
        self._searchGraspability \
            = rospy.ServiceProxy("search_graspability", asrv.searchGraspability)
        self._searchGraspabilityAction \
            = actionlib.SimpleActionClient(
                "/aist_graspability/search_graspability",
                amsg.searchGraspabilityAction)
        self._listener        = TransformListener()
        self._reference_frame = reference_frame

    def create_background_image(self, depth_topic):
        return self._createBackgroundImage(depth_topic).success

    def create_mask_image(self, depth_topic, nbins):
        return self._createMaskImage(depth_topic, nbins).success

    def search(self, camera_info_topic, depth_topic, normal_topic,
               gripper_type, part_id, bin_id):
        param = self._params[part_id]
        rospy.loginfo("search graspabilities for part_{}({}) in bin_{}"
                      .format(part_id, param.name, bin_id))

        try:
            (self._K, self._D) = self._get_camera_intrinsics(camera_info_topic)
            if normal_topic == "":
                self._normals = None
            else:
                self._normals = rospy.wait_for_message(normal_topic,
                                                       smsg.Image, timeout=10.0)
            res = self._searchGraspability(
                        depth_topic, bin_id, gripper_type,
                        param.ns,               param.detect_edge,
                        param.object_size,      param.radius,
                        param.open_width,       param.finger_width,
                        param.finger_thickness, param.insertion_depth)
            return (self._compute_poses(res), res.gscore, res.success)
        except Exception as e:
            rospy.logerr(e.message)
            return (None, None, None, False)

    def send_goal(self, camera_info_topic, depth_topic, normal_topic,
                  gripper_type, part_id, bin_id):
        param = self._params[part_id]
        rospy.loginfo("search graspabilities for part_{}({}) in bin_{}"
                      .format(part_id, param.name, bin_id))

        try:
            (self._K, self._D) = self._get_camera_intrinsics(camera_info_topic)
            if normal_topic == "":
                self._normals = None
            else:
                self._normals = rospy.wait_for_message(normal_topic,
                                                       smsg.Image, timeout=10.0)
        except Exception as e:
            rospy.logerr(e.message)
            return

        goal = amsg.searchGraspabilityGoal()
        goal.depth_topic      = depth_topic
        goal.bin_id           = bin_id
        goal.gripper_type     = gripper_type
        goal.ns               = param.ns
        goal.detect_edge      = param.detect_edge
        goal.object_size      = param.object_size
        goal.radius           = param.radius
        goal.open_width       = param.open_width
        goal.finger_width     = param.finger_width
        goal.finger_thickness = param.finger_thickness
        goal.insertion_depth  = param.insertion_depth
        self._searchGraspabilityAction.send_goal(goal,
                                                 feedback_cb=self._feedback_cb)

    def wait_for_result(self):
        try:
            self._searchGraspabilityAction.wait_for_result()
            result = self._searchGraspabilityAction.get_result()
            return (self._compute_poses(result), result.gscore, result.success)
        except Exception as e:
            rospy.logerr(e.message)
            return (None, None, None, False)

    def cancel_goal(self):
        self._searchGraspabilityAction.cancel_goal()

    def _feedback_cb(self, feedback):
        pass

    def _get_camera_intrinsics(self, camera_info_topic):
        camera_info = rospy.wait_for_message(camera_info_topic,
                                             smsg.CameraInfo, timeout=10.0)
        return (np.array(camera_info.K).reshape((3, 3)),
                np.array(camera_info.D))

    def _compute_poses(self, res):
        header = std_msgs.msg.Header()
        header.stamp    = res.header.stamp
        header.frame_id = self._reference_frame
        up = self._listener.asMatrix(res.header.frame_id, header)[0:3, 2]

        poses = []
        if self._normals is None:
            for i, uvd in enumerate(res.pos3D):
                rot = -radians(res.rotipz[i])
                poses.append(gmsg.PoseStamped(
                    res.header,
                    gmsg.Pose(gmsg.Point(*self._back_project_pixel(uvd)),
                              gmsg.Quaternion(*self._get_rotation(up, rot)))))
        else:
            bridge  = CvBridge()
            normals = bridge.imgmsg_to_cv2(self._normals, "32FC3")
            for i, uvd in enumerate(res.pos3D):
                nrm = self._fix_normal(normals[int(uvd.y), int(uvd.x), :],
                                       up, pi/4)
                rot = -radians(res.rotipz[i])
                poses.append(gmsg.PoseStamped(
                    res.header,
                    gmsg.Pose(gmsg.Point(*self._back_project_pixel(uvd)),
                              gmsg.Quaternion(*self._get_rotation(nrm, rot)))))

        return poses

    def _back_project_pixel(self, uvd):
        xy = cv2.undistortPoints(np.array([[[uvd.x, uvd.y]]], dtype=np.float32),
                                 self._K, self._D)[0, 0]
        return np.array((xy[0]*uvd.z, xy[1]*uvd.z, uvd.z))

    def _fix_normal(self, normal, up, max_slant):
        a = np.dot(normal, up)
        b = cos(max_slant)
        if a < b:
            p = sqrt((1.0 - b*b)/(1.0 - a*a))
            q = b - a*p
            return p*normal + q*up
        else:
            return normal

    def _get_rotation(self, normal, theta):
        c = cos(theta)
        s = sin(theta)
        k = (normal[0]*c + normal[1]*s)/normal[2]
        r = 1.0/sqrt(1.0 + k*k)
        R = np.identity(4, dtype=np.float32)
        R[0:3, 2] = normal
        R[0:3, 0] = (r*c, r*s, -r*k)
        R[0:3, 1] = np.cross(R[0:3, 2], R[0:3, 0])
        return tfs.quaternion_from_matrix(R)
