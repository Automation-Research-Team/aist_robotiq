import sys
import os
import rospy
import numpy as np
import cv2
from sensor_msgs import msg as smsg
from geometry_msgs import msg as gmsg
from aist_graspability import srv as asrv

######################################################################
#  class BinProperty                                                 #
######################################################################
class BinProperty(object):
    def __init__(self, name):
        self._name       = name

    @property
    def name(self):
        return self._name


######################################################################
#  class PartProperty                                               #
######################################################################
class PartProperty(object):
    def __init__(self, name, radius, obj_size, open_width, insertion_depth, ns):
        self._name            = name
        self._radius          = radius       # radius of the suction pad(pixel)
        self._obj_size        = obj_size     # approximate size of part
        self._open_width      = open_width   # open width of the gripper(pixel)
        self._insertion_depth = insertion_depth
        self._ns              = ns           # filter size for erosion

    @property
    def name(self):
        return self._name

    @property
    def radius(self):
        return self._radius

    @property
    def obj_size(self):
        return self._obj_size

    @property
    def open_width(self):
        return self._open_width

    @property
    def insertion_depth(self):
        return self._insertion_depth

    @property
    def ns(self):
        return self._ns

######################################################################
#  class GraspabilityClient                                          #
######################################################################
class GraspabilityClient(object):

    part_props = { # name, radius, obj_size, open_with, insertion_depth, ns
        4  : PartProperty("Geared motor",                    8, 15, 30, 1, 2),
        5  : PartProperty("Pully for round belt",            6,  8, 45, 3, 2),
        6  : PartProperty("Polyurethane round belt",         2,  2, 20, 5, 2),
        7  : PartProperty("Bearing wirh housing",           12, 12, 50, 1, 2),
        8  : PartProperty("Drive shaft",                     2,  4, 20, 4, 0),
        9  : PartProperty("End cap for shaft",               3,  3, 20, 1, 2),
        10 : PartProperty("Bearing spacers for inner ring",  3,  3, 30, 1, 2),
        11 : PartProperty("Pully for round belts clamping", 12, 12, 20, 1, 2),
        12 : PartProperty("Bearing spacer for inner ring",   2,  4, 20, 1, 2),
        13 : PartProperty("Idler for round belt",            2,  7, 30, 1, 2),
        14 : PartProperty("Bearing shaft screw",             2,  4, 14, 5, 0),
        15 : PartProperty("Hex nut",                         3,  3, 20, 1, 2),
        16 : PartProperty("Flat washer",                     3,  3, 15, 1, 0),
        17 : PartProperty("Head cap screw M4",               1,  1, 10, 1, 0),
        18 : PartProperty("Head cap scres M3",               1,  1, 10, 1, 0),
    }

    def __init__(self):
        super(GraspabilityClient, self).__init__()
        self.createMaskImage    = rospy.ServiceProxy("create_mask_image",
                                                     asrv.createMaskImage)
        self.searchGraspability = rospy.ServiceProxy("search_graspability",
                                                     asrv.searchGraspability)

    def create_mask_image(self, image_topic, image_dir):
        return self.createMaskImage(image_topic, image_dir).success

    def search(self, camera_info_topic, image_topic, gripper_type,
               part_id, bin_id, image_dir=""):
        part_prop = GraspabilityClient.part_props[part_id]
        rospy.loginfo("search graspabilities for " + part_prop.name)

        try:
            (K, D) = self._get_camera_intrinsics(camera_info_topic)
            res    = self.searchGraspability(image_topic, gripper_type,
                                             part_id, bin_id, image_dir)

            poses = gmsg.PoseArray()
            poses.header = res.header
            for i in range(len(res.pos3D)):
                p = self._back_project_pixel(res.pos3D[i], K, D)
                poses.poses.append(gmsg.Pose(gmsg.Point(*p),
                                             gmsg.Quaternion(0, 0, 0, 1)))
            return (poses, res.rotipz, res.gscore, res.success)

        except rospy.ROSException:
            rospy.logerr("wait_for_message(): Timeout expired!")
            return (None, None, None, False)

    def _get_camera_intrinsics(self, camera_info_topic):
        camera_info = rospy.wait_for_message(camera_info_topic,
                                             smsg.CameraInfo, timeout=10.0)
        return (np.array(camera_info.K).reshape((3, 3)),
                np.array(camera_info.D))

    def _back_project_pixel(self, uvd, K, D):
        rospy.logdebug("pixel point and depth: %f, %f, %f" % (uvd.x, uvd.y, uvd.z))
        # z = d
        # x = (u - self.cx) * d / self.fx
        # y = (v - self.cy) * d / self.fy
        distorted_uv   = np.array([[[uvd.x, uvd.y]]], dtype=np.float32)
        undistorted_xy = cv2.undistortPoints(distorted_uv, K, D)
        normalized_x   = undistorted_xy[0, 0, 0]
        normalized_y   = undistorted_xy[0, 0, 1]
        z = uvd.z
        x = normalized_x * z
        y = normalized_y * z
        rospy.logdebug("spatial position: %f, %f, %f" % (x, y, z))
        return x, y, z
