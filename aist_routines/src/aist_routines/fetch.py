import rospy
import actionlib
import geometry_msgs.msg as gmsg
import tf.transformations as tfs

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
    def __init__(self):
        super(FetchRoutines, self).__init__()

        self._follow_trajectory = actionlib.SimpleActionClient(
                                    "head_controller/follow_joint_trajectory",
                                    FollowJointTrajectoryAction)
        self._follow_trajectory.wait_for_server()
        self._point_head = actionlib.SimpleActionClient(
                                "head_controller/point_head", PointHeadAction)
        self._point_head.wait_for_server()
        rospy.loginfo("Connected to head_controller.")

    def move_head(self, pan, tilt, duration=5.0, wait=True):
        trajectory = JointTrajectory()
        trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions       = [pan, tilt]
        trajectory.points[0].velocities      = [0.0, 0.0]
        trajectory.points[0].accelerations   = [0.0, 0.0]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._follow_trajectory.send_goal(goal)
        if wait:
            self._follow_trajectory.wait_for_result()
            return self._follow_trajectory.get_result().error_code
        else:
            return FollowJointTrajectory.SUCCESFUL

    def shake_head(self, mag_pan, mag_tilt,
                   nsteps=20, duration=1.0, wait=True):
        trajectory = JointTrajectory()
        trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]
        for i in range(nsteps + 1):
            pan  = mag_pan  * sin(4.0*pi*i/nsteps)
            tilt = mag_tilt * sin(2.0*pi*i/nsteps)
            # print("i={}, pan={}, tilt={}".format(i, pan, tilt))
            trajectory.points.append(JointTrajectoryPoint())
            trajectory.points[i].positions       = [pan, tilt]
            trajectory.points[i].velocities      = [0.0, 0.0]
            trajectory.points[i].accelerations   = [0.0, 0.0]
            trajectory.points[i].time_from_start = rospy.Duration((i + 1)
                                                                  *duration)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self._follow_trajectory.send_goal(goal)
        if wait:
            self._follow_trajectory.wait_for_result()
            return self._follow_trajectory.get_result().error_code
        else:
            return FollowJointTrajectory.SUCCESFUL

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
