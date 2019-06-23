import sys, os, collections
import rospy
import numpy as np
import cv2
from math              import radians, cos, sin, sqrt
from sensor_msgs       import msg as smsg
from geometry_msgs     import msg as gmsg
from aist_graspability import srv as asrv
from cv_bridge         import CvBridge, CvBridgeError
from tf                import transformations as tfs

######################################################################
#  class GraspabilityClient                                          #
######################################################################
class GraspabilityClient(object):
    PartProps = collections.namedtuple(
        "PartProps", "name, radius obj_size, open_width, insertion_depth, ns")
    _part_props = {
        4  : PartProps("Geared motor",                    8, 15, 30, 1, 2),
        5  : PartProps("Pully for round belt",            6,  8, 45, 3, 2),
        6  : PartProps("Polyurethane round belt",         2,  2, 20, 5, 2),
        7  : PartProps("Bearing wirh housing",           12, 12, 50, 1, 2),
        8  : PartProps("Drive shaft",                     2,  4, 20, 4, 0),
        9  : PartProps("End cap for shaft",               3,  3, 20, 1, 2),
        10 : PartProps("Bearing spacers for inner ring",  3,  3, 30, 1, 2),
        11 : PartProps("Pully for round belts clamping", 12, 12, 20, 1, 2),
        12 : PartProps("Bearing spacer for inner ring",   2,  4, 20, 1, 2),
        13 : PartProps("Idler for round belt",            2,  7, 30, 1, 2),
        14 : PartProps("Bearing shaft screw",             2,  4, 14, 5, 0),
        15 : PartProps("M6 hex nut",                      3,  3, 20, 1, 2),
        16 : PartProps("M6 flat washer",                  3,  3, 15, 1, 0),
        17 : PartProps("M4 head cap screw",               1,  1, 10, 1, 0),
        18 : PartProps("M3 head cap scres",               1,  1, 10, 1, 0),
    }

    def __init__(self):
        super(GraspabilityClient, self).__init__()
        self._createBackgroundImage \
            = rospy.ServiceProxy("create_background_image",
                                 asrv.createBackgroundImage)
        self._createMaskImage    = rospy.ServiceProxy("create_mask_image",
                                                      asrv.createMaskImage)
        self._searchGraspability = rospy.ServiceProxy("search_graspability",
                                                      asrv.searchGraspability)

    def create_background_image(self, depth_topic, image_dir=""):
        return self._createBackgroundImage(depth_topic, image_dir).success

    def create_mask_image(self, depth_topic, nbins, image_dir=""):
        return self._createMaskImage(depth_topic, nbins, image_dir).success

    def search(self, camera_info_topic, depth_topic, normal_topic,
               gripper_type, part_id, bin_id, image_dir=""):
        part_prop = GraspabilityClient._part_props[part_id]
        rospy.loginfo("search graspabilities for part_{}({}) in bin_{}"
                      .format(part_id, part_prop.name, bin_id))

        try:
            (K, D) = self._get_camera_intrinsics(camera_info_topic)
            res    = self._searchGraspability(depth_topic, gripper_type,
                                              part_id, bin_id, image_dir)
            poses = []
            if normal_topic == "":
                for uvd in res.pos3D:
                    xyz = self._back_project_pixel(uvd, K, D)
                    poses.append(gmsg.PoseStamped(
                        res.header,
                        gmsg.Pose(gmsg.Point(*xyz),
                                  gmsg.Quaternion(1, 0, 0, 0))))
            else:
                bridge  = CvBridge()
                normals = bridge.imgmsg_to_cv2(
                              rospy.wait_for_message(normal_topic,
                                                     smsg.Image, timeout=10.0),
                              "32FC3")
                for i, uvd in enumerate(res.pos3D):
                    xyz = self._back_project_pixel(uvd, K, D)
                    n   = normals[int(uvd.y), int(uvd.x), :]
                    rot = -radians(res.rotipz[i])
                    poses.append(gmsg.PoseStamped(
                        res.header,
                        gmsg.Pose(gmsg.Point(*xyz),
                                  gmsg.Quaternion(*self._get_rotation(n,
                                                                      rot)))))
                    print("normal = {}".format(n))

            return (poses, res.rotipz, res.gscore, res.success)

        except rospy.ROSException:
            rospy.logerr("wait_for_message(): Timeout expired!")
            return (None, None, None, False)

        except CvBridgeError as e:
            rospy.logerr(e)
            return (None, None, None, False)

    def _get_camera_intrinsics(self, camera_info_topic):
        camera_info = rospy.wait_for_message(camera_info_topic,
                                             smsg.CameraInfo, timeout=10.0)
        return (np.array(camera_info.K).reshape((3, 3)),
                np.array(camera_info.D))

    def _back_project_pixel(self, uvd, K, D):
        xy = cv2.undistortPoints(np.array([[[uvd.x, uvd.y]]], dtype=np.float32),
                                 K, D)[0, 0]
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
