import rospy
import actionlib
import geometry_msgs.msg  as gmsg
import sensor_msgs.msg    as smsg
import tf.transformations as tfs
import threading
import copy

from numpy               import clip
from math                import radians, degrees, cos, sin, pi
from control_msgs.msg    import (FollowJointTrajectoryAction,
                                 FollowJointTrajectoryGoal,
                                 PointHeadAction, PointHeadGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from freight             import FreightRoutines

######################################################################
#  class FetchRoutines                                               #
######################################################################
class FetchRoutines(FreightRoutines):
    class FollowTrajectoryClient(object):
        """
        Send a trajectory to controller
        """

        def __init__(self, name, joint_names):
            self._client = actionlib.SimpleActionClient(
                "%s/follow_joint_trajectory" % name,
                FollowJointTrajectoryAction)
            rospy.loginfo("Waiting for %s..." % name)
            self._client.wait_for_server()
            self._joint_names = joint_names

        def move_to(self, positions, duration):
            if len(self._joint_names) != len(positions):
                rospy.logerr("Invalid trajectory position")
                return False

            trajectory = JointTrajectory()
            trajectory.joint_names = self._joint_names
            trajectory.points.append(JointTrajectoryPoint())
            trajectory.points[0].positions       = positions
            trajectory.points[0].velocities      = [0.0 for _ in positions]
            trajectory.points[0].accelerations   = [0.0 for _ in positions]
            trajectory.points[0].time_from_start = rospy.Duration(duration)
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = trajectory
            self._client.send_goal(goal)

            self._client.wait_for_result()
            res = self._client.get_result()
            if res.error_code != 0:
                rospy.logerr(res.error_code)
                return False
            return True

        def follow(self, positions_list, duration):
            time_step  = duration/len(positions_list)
            trajectory = JointTrajectory()
            trajectory.joint_names = self._joint_names
            for i, positions in enumerate(positions_list):
                trajectory.points.append(JointTrajectoryPoint())
                trajectory.points[i].positions       = positions
                trajectory.points[i].velocities      = [0.0 for _ in positions]
                trajectory.points[i].accelerations   = [0.0 for _ in positions]
                trajectory.points[i].time_from_start = rospy.Duration(
                                                        (i + 1)*time_step)
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = trajectory
            self._client.send_goal(goal)

            self._client.wait_for_result()
            res = self._client.get_result()
            if res.error_code != 0:
                rospy.logerr(res.error_code)
                return False
            return True


    def __init__(self):
        super(FetchRoutines, self).__init__()

        self._lock = threading.Lock()
        self._joint_names     = None
        self._joint_positions = None
        rospy.Subscriber("/joint_states", smsg.JointState,
                         self._joint_states_cb)

        self._torso_controller = FetchRoutines.FollowTrajectoryClient(
            "torso_controller", ["torso_lift_joint"])
        self._head_controller = FetchRoutines.FollowTrajectoryClient(
            "head_controller", ["head_pan_joint", "head_tilt_joint"])
        self._point_head = actionlib.SimpleActionClient(
                                "head_controller/point_head", PointHeadAction)
        self._point_head.wait_for_server()

    @property
    def torso_position(self):
        return self._torso_controller.current_position

    @property
    def head_position(self):
        return self._head_controller.current_position

    def move_torso(self, position, duration=5.0):
        return self._torso_controller.move_to([clip(position, 0.0, 0.4)],
                                              duration)

    def move_head(self, pan, tilt, duration=5.0):
        return self._head_controller.move_to([pan, tilt], duration)

    def shake_head(self, mag_pan, mag_tilt, nsteps=20, duration=10.0):
        positions_list = []
        for i in range(nsteps + 1):
            pan  = mag_pan  * sin(4.0*pi*i/nsteps)
            tilt = mag_tilt * sin(2.0*pi*i/nsteps)
            positions_list.append([pan, tilt])
        return self._head_controller.follow(positions_list, duration)

    def gaze(self, target_point):
        goal = PointHeadGoal()
        goal.target.header.stamp    = rospy.Time.now()
        goal.target.header.frame_id = target_point.header.frame_id
        goal.target.point           = target_point.point
        goal.min_duration           = rospy.Duration(1.0)
        self._point_head.send_goal(goal)
        self._point_head.wait_for_result()

    def gaze_frame(self, target_frame):
        target_point = gmsg.PointStamped()
        target_point.header.frame_id = target_frame
        target_point.point           = gmsg.Point(0, 0, 0)
        self.gaze(target_point)

    def _current_position(self, joint_name):
        try:
            index = self._joint_names.index(joint_name)
        except ValueError:
            return None

        self._lock.acquire()
        position = copy.deepcopy(self._joint_positions[index])
        self._lock.release()
        return position

    def _joint_states_cb(self, msg):
        self._lock.acquire()
        self._joint_names     = copy.deepcopy(msg.name)
        self._joint_positions = copy.deepcopy(msg.position)
        self._lock.release()
