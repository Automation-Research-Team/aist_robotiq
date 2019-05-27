import sys
import os
import rospy
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
#  class PartsProperty                                               #
######################################################################
class PartsProperty(object):
    def __init__(self, name, radius, obj_size, open_width, insertion_depth, ns):
        self._name            = name
        self._radius          = radius       # radius of the suction pad(pixel)
        self._obj_size        = obj_size     # approximate size of parts
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

    parts_props = { # name, radius, obj_size, open_with, insertion_depth, ns
        4  : PartsProperty("Geared motor",                    8, 15, 30, 1, 2),
        5  : PartsProperty("Pully for round belt",            6,  8, 45, 3, 2),
        6  : PartsProperty("Polyurethane round belt",         2,  2, 20, 5, 2),
        7  : PartsProperty("Bearing wirh housing",           12, 12, 50, 1, 2),
        8  : PartsProperty("Drive shaft",                     2,  4, 20, 4, 0),
        9  : PartsProperty("End cap for shaft",               3,  3, 20, 1, 2),
        10 : PartsProperty("Bearing spacers for inner ring",  3,  3, 30, 1, 2),
        11 : PartsProperty("Pully for round belts clamping", 12, 12, 20, 1, 2),
        12 : PartsProperty("Bearing spacer for inner ring",   2,  4, 20, 1, 2),
        13 : PartsProperty("Idler for round belt",            2,  7, 30, 1, 2),
        14 : PartsProperty("Bearing shaft screw",             2,  4, 14, 5, 0),
        15 : PartsProperty("Hex nut",                         3,  3, 20, 1, 2),
        16 : PartsProperty("Flat washer",                     3,  3, 15, 1, 0),
        17 : PartsProperty("Head cap screw M4",               1,  1, 10, 1, 0),
        18 : PartsProperty("Head cap scres M3",               1,  1, 10, 1, 0),
    }

    def __init__(self):
        super(GraspabilityClient, self).__init__()
        self.createMaskImage    = rospy.ServiceProxy("create_mask_image",
                                                     asrv.createMaskImage)
        self.searchGraspability = rospy.ServiceProxy("search_graspability",
                                                     asrv.searchGraspability)

    def create_mask_image(self, image_topic, dir_path):
        req = asrv.createMaskImage()
        req.image_topic = image_topic
        req.dir_path    = dir_path
        return self.createMaskImage.call(req)

    def search(self, camera_info_topic, image_topic, part_id, bin_id,
               gripper_type, dir_path):
        parts_prop = parts_props[part_id]
        rospy.loginfo("search graspabilities for " + parts_prop.name)

        try:
            (K, D) = self._get_camera_intrinsics(camera_info_topic)

            req = asrv.searchGraspabilityRequest()
            req.image_topic  = image_topic
            req.part_id      = part_id
            req.bin_id       = bin_id
            req.gripper_type = gripper_type
            req.dir_path     = dir_path
            res = self.searchGraspability.call(req)
        except rospy.ROSException:
            rospy.logerr("wait_for_message(): Timeout expired!")
            return False
        return True

    def _get_camera_intrinsics(camera_info_topic):
        cinfo_msg = rospy.wait_for_message(camera_info_topic,
                                           smsg.Camerainfo, timeout=10.0)
        return (np.array(cinfo_msg.K).reshape((3, 3)), np.array(cinfo_msg.D))


    def _back_project_pixel(self, u, v, d, K, D):
        """
        convert graspability result pixel value to distance.
        """

        rospy.logdebug("pixel point and depth: %f, %f, %f" % (u, v, d))
        # z = d
        # x = (u - self.cx) * d / self.fx
        # y = (v - self.cy) * d / self.fy
        distorted_uv = np.array([[[u,v]]], dtype=np.float32)
        undistorted_xy = cv2.undistortPoints(distorted_uv, K, D)
        normalized_x = undistorted_xy[0,0,0]
        normalized_y = undistorted_xy[0,0,1]
        z = d
        x = normalized_x * z
        y = normalized_y * z
        rospy.logdebug("spatial position: %f, %f, %f" % (x, y, z))
        return x, y, z

    def _take_new_image(self, file_path):
        """Update scene image for graspability estimation."""
        while not self._trigger_frame_client().out:
            rospy.logerr("Failed to trig new frame.")
        rospy.sleep(2)
        if not self._get_frame_client(0, True).success:
            rospy.logerr("Failed to get frame data.")
            return False

        skimage.io.imsave(file_path, self._depth_image)

        return True
