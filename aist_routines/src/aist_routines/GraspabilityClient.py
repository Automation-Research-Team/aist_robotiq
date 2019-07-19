import sys, os, collections
import rospy
import actionlib
import numpy as np
import cv2
import std_msgs
from math              import radians, cos, sin, sqrt
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
    PartProps = collections.namedtuple(
        "PartProps", "name, ns, detect_edge, object_size, radius, open_width, finger_width, finger_thickness, insertion_depth")
    _part_props = {
        4  : PartProps("Geared motor",
                       2, True,  15,  8, 100, 1, 5, 10),
        5  : PartProps("Pully for round belt",
                       2, True,   8,  6,  45, 1, 5,  3),
        6  : PartProps("Polyurethane round belt",
                       2, True,   2,  2,  20, 1, 5,  5),
        7  : PartProps("Bearing wirh housing",
                       2, True,  12, 12,  20, 1, 5,  5),
        8  : PartProps("Drive shaft",
                       0, False,  4,  2,  20, 1, 5,  4),
        9  : PartProps("End cap for shaft",
                       2, True,   3,  3,  20, 1, 5,  1),
        10 : PartProps("Bearing spacers for inner ring",
                       2, True,   3,  3,  20, 1, 5,  1),
        11 : PartProps("Pully for round belts clamping",
                       2, True,  12, 12,  20, 1, 5,  1),
        12 : PartProps("Bearing spacer for inner ring",
                       2, True,   4,  2,  20, 1, 5,  1),
        13 : PartProps("Idler for round belt",
                       2, True,   7,  2,  30, 1, 5,  1),
        14 : PartProps("Bearing shaft screw",
                       0, True,   4,  2,  14, 1, 5,  5),
        15 : PartProps("M6 hex nut",
                       2, True,   3,  3,  20, 1, 5,  1),
        16 : PartProps("M6 flat washer",
                       0, True,   3,  3,  15, 1, 5,  1),
        17 : PartProps("M4 head cap screw",
                       0, True,   1,  1,  10, 1, 5,  1),
        18 : PartProps("M3 head cap scres",
                       0, True,   1,  1,  10, 1, 5,  1),
    }

    def __init__(self):
        super(GraspabilityClient, self).__init__()
        self._createBackgroundImage \
            = rospy.ServiceProxy("/aist_graspability/create_background_image",
                                 asrv.createBackgroundImage)
        self._createMaskImage \
            = rospy.ServiceProxy("create_mask_image", asrv.createMaskImage)
        self._searchGraspability \
            = rospy.ServiceProxy("search_graspability", asrv.searchGraspability)
        self._searchGraspabilityAction \
            = actionlib.SimpleActionClient(
                "/aist_graspability/search_graspability",
                amsg.searchGraspabilityAction)
        self._listener = TransformListener()

    def create_background_image(self, depth_topic):
        return self._createBackgroundImage(depth_topic).success

    def create_mask_image(self, depth_topic, nbins):
        return self._createMaskImage(depth_topic, nbins).success

    def search(self, camera_info_topic, depth_topic, normal_topic,
               gripper_type, part_id, bin_id):
        part_prop = GraspabilityClient._part_props[part_id]
        rospy.loginfo("search graspabilities for part_{}({}) in bin_{}"
                      .format(part_id, part_prop.name, bin_id))

        try:
            (self._K, self._D) = self._get_camera_intrinsics(camera_info_topic)
            res = self._searchGraspability(
                        depth_topic, bin_id, gripper_type,
                        part_prop.ns,               part_prop.detect_edge,
                        part_prop.object_size,      part_prop.radius,
                        part_prop.open_width,       part_prop.finger_width,
                        part_prop.finger_thickness, part_prop.insertion_depth)
            poses = []
            if normal_topic == "":
                self._normals = None
                header = std_msgs.msg.Header()
                header.stamp    = res.header.stamp
                header.frame_id = "workspace_center"
                T   = self._listener.asMatrix(res.header.frame_id, header)
                nrm = T[0:3, 2]
                for i, uvd in enumerate(res.pos3D):
                    rot = -radians(res.rotipz[i])
                    poses.append(gmsg.PoseStamped(
                        res.header,
                        gmsg.Pose(
                            gmsg.Point(*self._back_project_pixel(uvd)),
                            gmsg.Quaternion(*self._get_rotation(nrm, rot)))))
            else:
                bridge  = CvBridge()
                normals = bridge.imgmsg_to_cv2(
                              rospy.wait_for_message(normal_topic,
                                                     smsg.Image, timeout=10.0),
                              "32FC3")
                for i, uvd in enumerate(res.pos3D):
                    nrm = normals[int(uvd.y), int(uvd.x), :]
                    rot = -radians(res.rotipz[i])
                    poses.append(gmsg.PoseStamped(
                        res.header,
                        gmsg.Pose(
                            gmsg.Point(*self._back_project_pixel(uvd)),
                            gmsg.Quaternion(*self._get_rotation(nrm, rot)))))

            return (poses, res.gscore, res.success)

        except rospy.ROSException as e:
            rospy.logerr(e.message)
            return (None, None, None, False)

        except CvBridgeError as e:
            rospy.logerr(e)
            return (None, None, None, False)

    def send_goal(self, camera_info_topic, depth_topic, normal_topic,
                  gripper_type, part_id, bin_id):
        part_prop = GraspabilityClient._part_props[part_id]
        rospy.loginfo("search graspabilities for part_{}({}) in bin_{}"
                      .format(part_id, part_prop.name, bin_id))

        try:
            (self._K, self._D) = self._get_camera_intrinsics(camera_info_topic)
            if normal_topic == "":
                self._normals = None
            else:
                self._normals = rospy.wait_for_message(normal_topic,
                                                       smsg.Image, timeout=10.0)
        except rospy.ROSException as e:
            rospy.logerr(e.message)
            return

        goal = amsg.searchGraspabilityGoal()
        goal.depth_topic      = depth_topic
        goal.bin_id           = bin_id
        goal.gripper_type     = gripper_type
        goal.ns               = part_prop.ns
        goal.detect_edge      = part_prop.detect_edge
        goal.object_size      = part_prop.object_size
        goal.radius           = part_prop.radius
        goal.open_width       = part_prop.open_width
        goal.finger_width     = part_prop.finger_width
        goal.finger_thickness = part_prop.finger_thickness
        goal.insertion_depth  = part_prop.insertion_depth
        self._searchGraspabilityAction.send_goal(goal,
                                                 feedback_cb=self._feedback_cb)

    def wait_for_result(self):
        self._searchGraspabilityAction.wait_for_result()
        result = self._searchGraspabilityAction.get_result()

        poses = []
        if self._normals is None:
            header = std_msgs.msg.Header()
            header.stamp    = result.header.stamp
            header.frame_id = "workspace_center"
            T   = self._listener.asMatrix(result.header.frame_id, header)
            nrm = T[0:3, 2]
            for i, uvd in enumerate(result.pos3D):
                rot = -radians(result.rotipz[i])
                poses.append(gmsg.PoseStamped(
                    result.header,
                    gmsg.Pose(gmsg.Point(*self._back_project_pixel(uvd)),
                              gmsg.Quaternion(*self._get_rotation(nrm, rot)))))
        else:
            bridge  = CvBridge()
            normals = bridge.imgmsg_to_cv2(self._normals, "32FC3")
            for i, uvd in enumerate(result.pos3D):
                nrm = normals[int(uvd.y), int(uvd.x), :]
                rot = -radians(result.rotipz[i])
                poses.append(gmsg.PoseStamped(
                    result.header,
                    gmsg.Pose(gmsg.Point(*self._back_project_pixel(uvd)),
                              gmsg.Quaternion(*self._get_rotation(nrm, rot)))))

        return (poses, result.gscore, result.success)

    def cancel_goal(self):
        self._searchGraspabilityAction.cancel_goal()

    def _feedback_cb(self, feedback):
        pass

    def _get_camera_intrinsics(self, camera_info_topic):
        camera_info = rospy.wait_for_message(camera_info_topic,
                                             smsg.CameraInfo, timeout=10.0)
        return (np.array(camera_info.K).reshape((3, 3)),
                np.array(camera_info.D))

    def _back_project_pixel(self, uvd):
        xy = cv2.undistortPoints(np.array([[[uvd.x, uvd.y]]], dtype=np.float32),
                                 self._K, self._D)[0, 0]
        return np.array((xy[0]*uvd.z, xy[1]*uvd.z, uvd.z))

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
