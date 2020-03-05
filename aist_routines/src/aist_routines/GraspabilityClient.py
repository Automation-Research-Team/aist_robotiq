import rospy, rospkg, rosparam
import base
from aist_graspability import GraspabilityClient as GraspabilityClientBase

######################################################################
#  class GraspabilityClient                                          #
######################################################################
class GraspabilityClient(GraspabilityClientBase):

    def __init__(self, reference_frame, name="aist_graspability"):
        super(GraspabilityClient, self).__init__(reference_frame, name)

        rospack = rospkg.RosPack()
        d = rosparam.load_file(rospack.get_path("aist_routines") +
                               "/config/graspability_parameters.yaml")[0][0]
        self._params = base.paramtuples(d["graspability_parameters"])

    def search(self, camera_info_topic, depth_topic, normal_topic,
               gripper_type, part_id, bin_id):
        param = self._params[part_id]
        rospy.loginfo("search graspabilities for part_{}({}) in bin_{}"
                      .format(part_id, param.name, bin_id))

        return super(GraspabilityClient, self).search(
                camera_info_topic, depth_topic, normal_topic, bin_id, gripper_type,
                param.ns, param.detect_edge, param.object_size, param.radius,
                param.open_width, param.finger_width, param.finger_thickness,
                param.insertion_depth)

    def send_goal(self, camera_info_topic, depth_topic, normal_topic,
                  gripper_type, part_id, bin_id):
        param = self._params[part_id]
        rospy.loginfo("search graspabilities for part_{}({}) in bin_{}"
                      .format(part_id, param.name, bin_id))

        super(GraspabilityClient, self).send_goal(
                camera_info_topic, depth_topic, normal_topic, bin_id, gripper_type,
                param.ns, param.detect_edge, param.object_size, param.radius,
                param.open_width, param.finger_width, param.finger_thickness,
                param.insertion_depth)
