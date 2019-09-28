import rospy
import actionlib
import geometry_msgs.msg as gmsg
import tf.transformations as tfs

from math                import radians, degrees
from control_msgs.msg    import (FollowJointTrajectoryAction,
                                 FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from base                import AISTBaseRoutines
#from freight             import FreightRoutines

######################################################################
#  class FetchRoutines                                               #
######################################################################
class FetchRoutines(AISTBaseRoutines):
    def __init__(self):
        super(FetchRoutines, self).__init__()

        self._head_controller = actionlib.SimpleActionClient(
                                    "head_controller/follow_joint_trajectory",
                                    FollowJointTrajectoryAction)
        self._head_controller.wait_for_server()
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
        self._head_controller.send_goal(goal)
        if wait:
            self._head_controller.wait_for_result()
            return self._head_controller.get_result().error_code
        else:
            return FollowJointTrajectory.SUCCESFUL

    def gaze(self, point):
        try:
            self.listener.waitForTransform("torso_lift_link",
                                           pose.header.frame_id,
                                           respy.Time.now(),
                                           rospy.Duration(10))
            p = self.listener.transformPoint("torso_lift_link", point).point
        except Excetion as e:
            rospy.logerr("FetchRoutines.gaze(): {}".format(e))
            return
