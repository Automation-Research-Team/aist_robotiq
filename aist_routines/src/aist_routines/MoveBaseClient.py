import copy
import threading
import rospy
import actionlib
import geometry_msgs.msg as gmsg
import tf.transformations as tfs

from math                import pi, radians, degrees
from move_base_msgs.msg  import MoveBaseAction, MoveBaseGoal
from nav_msgs            import msg as nmsg
from geometry_msgs       import msg as gmsg
from tf                  import TransformListener, transformations as tfs

######################################################################
#  class MoveBaseClient                                              #
######################################################################
class MoveBaseClient(object):
    def __init__(self):
        super(MoveBaseClient, self).__init__()

        self._move_base = actionlib.SimpleActionClient("move_base",
                                                       MoveBaseAction)
        if self._move_base.wait_for_server(rospy.Duration(20)):
            rospy.loginfo("Connected to move_base.")
        else:
            rospy.logerr("MoveBaseClient.__init()__: failed to connect move_base action server")

        self._odom_sub          = rospy.Subscriber("odom", nmsg.Odometry,
                                                   self._odom_callback)
        self._odom_recv_event   = threading.Event()
        self._current_odom      = nmsg.Odometry()
        self._current_odom.header.stamp = rospy.Time.now()
        self._current_odom.header.frame_id = "odom"
        self._reference_frame   = "map"
        self._listener          = TransformListener()

    @property
    def current_odom(self):
        return self._current_odom

    def move_base(self, x, y, theta, frame="map", wait=True):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id  = frame
        goal.target_pose.pose.position    = gmsg.Vector3(x, y, 0)
        goal.target_pose.pose.orientation = gmsg.Quaternion(
            *tfs.quaternion_from_euler(0, 0, theta))
        return self._move_base.send_goal_and_wait(goal)

    def move_base_to_frame(self, target_frame):
        return self.move_base(0, 0, 0, target_frame)

    def format_odom(self, odom):
        xyzrpy = self._xyz_rpy(odom)
        return "[{:.4f}, {:.4f} ; {:.2f}]".format(xyzrpy[0], xyzrpy[1],
                                                  degrees(xyzrpy[5]))

    def _xyz_rpy(self, odom):
        try:
            pose = gmsg.PoseStamped()
            pose.header = odom.header
            pose.pose   = odom.pose.pose
            self._listener.waitForTransform(self._reference_frame,
                                            pose.header.frame_id,
                                            pose.header.stamp,
                                            rospy.Duration(10))
            transformed_pose = self._listener.transformPose(
                                self._reference_frame, pose).pose
        except Exception as e:
            rospy.logerr("MoveBaseClient._xyz_rpy(): {}".format(e))
            raise e

        rpy = tfs.euler_from_quaternion([transformed_pose.orientation.x,
                                         transformed_pose.orientation.y,
                                         transformed_pose.orientation.z,
                                         transformed_pose.orientation.w])
        return [transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z,
                rpy[0], rpy[1], rpy[2]]

    def _odom_callback(self, odom):
        self._odom_recv_event.clear()
        self._current_odom = copy.deepcopy(odom)
        self._odom_recv_event.set()
