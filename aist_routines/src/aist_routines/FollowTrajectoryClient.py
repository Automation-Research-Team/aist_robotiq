import rospy
import actionlib

from control_msgs.msg    import (FollowJointTrajectoryAction,
                                 FollowJointTrajectoryGoal,
                                 PointHeadAction, PointHeadGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

######################################################################
#  class FollowTrajectoryClinet                                      #
######################################################################
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
