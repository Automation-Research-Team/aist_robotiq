import copy
import threading
import rospy
import actionlib
import geometry_msgs.msg as gmsg
import tf.transformations as tfs

from math                import radians, degrees
from move_base_msgs.msg  import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg        import Odometry

from base                import AISTBaseRoutines

######################################################################
#  class FreightRoutines                                             #
######################################################################
class FreightRoutines(AISTBaseRoutines):
    def __init__(self):
        super(FreightRoutines, self).__init__()

        self._move_base = actionlib.SimpleActionClient("/move_base",
                                                       MoveBaseAction)
        if self._move_base.wait_for_server(rospy.Duration(10)):
            rospy.loginfo("Connected to move_base.")
        else:
            rospy.logerr("FreightRoutines::__init()__: failed to connect /move_base action server")

        self._odom_sub          = rospy.Subscriber("/odom", Odometry,
                                                   self._odom_callback)
        self._odom_recv_event   = threading.Event()
        self._current_base_pose = gmsg.PoseStamped()

    @property
    def current_base_pose(self):
        return self._current_base_pose

    def move_base(self, x, y, theta, frame="map"):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id  = frame
        goal.target_pose.pose.position    = gmsg.Vector3(x, y, 0)
        goal.target_pose.pose.orientation = gmsg.Quaternion(
            *tfs.quaternion_from_euler(0, 0, theta))
        return self._move_base.send_goal_and_wait(goal)

    def _odom_callback(self, odom):
        self._odom_recv_event.clear()
        self._current_base_pose = copy.deepcopy(odom.pose)
        self._odom_recv_event.set()
