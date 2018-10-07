import rospy
import tf
from geometry_msgs.msg import Quaternion Vector3
from easy_handeye.handeye_calibration import HandeyeCalibration

calib = HandeyeCalibration()
calib.from_file()

if calib.eye_on_hand:
    overriding_robot_effector_frame = rospy.get_param('robot_effector_frame')
    if overriding_robot_effector_frame != "":
        calib.transformation.header.frame_id = overriding_robot_effector_frame
else:
    overriding_robot_base_frame = rospy.get_param('robot_base_frame')
    if overriding_robot_base_frame != "":
        calib.transformation.header.frame_id = overriding_robot_base_frame
overriding_tracking_base_frame = rospy.get_param('tracking_base_frame')

if overriding_tracking_base_frame != "":
    calib.transformation.child_frame_id = overriding_tracking_base_frame

rospy.loginfo('loading calibration parameters into namespace {}'.format(rospy.get_namespace()))
calib.to_parameters()


def rpy_to_quaternion(rpy):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(rpy.x, rpy.y, rpy.z)

    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def quaternion_to_rpy(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    rpy = tf.transformations.euler_from_quaternion((quaternion.x,
                                                    quaternion.y,
                                                    quaternion.z,
                                                    quaternion.w))
    return Vector3(x=rpy[0], y=rpy[1], z=rpy[2])
