import rospy
import actionlib
from geometry_msgs import msg as gmsg
from aist_msgs     import msg as amsg
from tf            import transformations as tfs

######################################################################
#  class PickOrPlaceAction                                           #
######################################################################
class PickOrPlaceAction(object):
    def __init__(self, routines):
        super(PickOrPlaceAction, self).__init__()

        self._routines = routines
        self._server = actionlib.SimpleActionServer("pickOrPlace",
                                                    amsg.pickOrPlaceAction,
                                                    execute_cb=self._callback,
                                                    auto_start=False)
        self._server.start()
        self._client = actionlib.SimpleActionClient('pickOrPlace',
                                                    amsg.pickOrPlaceAction)
        self._client.wait_for_server()

    def execute(self, robot_name, pose_stamped, pick, gripper_command,
                grasp_offset, approach_offset, liftup_after,
                speed_fast, speed_slow, acc_fast, acc_slow):
        goal = amsg.pickOrPlaceGoal()
        goal.robot_name      = robot_name
        goal.pose            = pose_stamped
        goal.pick            = pick
        goal.gripper_command = gripper_command
        goal.grasp_offset    = grasp_offset
        goal.approach_offset = approach_offset
        goal.liftup_after    = liftup_after
        goal.speed_fast      = speed_fast
        goal.speed_slow      = speed_slow
        goal.acc_fast        = acc_fast
        goal.acc_slow        = acc_slow
        self._client.send_goal(goal)
        self._client.wait_for_result()
        return self._client.get_result().success

    def _callback(self, goal):
        routines = self._routines
        gripper  = routines.gripper(goal.robot_name)
        feedback = amsg.pickOrPlaceFeedback()

        # Go to approach pose.
        rospy.loginfo("Go to approach pose.")
        (_, _, feedback.current_pose) \
            = routines.go_to_pose_goal(goal.robot_name,
                                       routines.effector_target_pose(
                                           goal.pose,
                                           (0, 0, goal.approach_offset)),
                                       goal.speed_fast)
        self._server.publish_feedback(feedback)

        # Pregrasp
        if goal.pick:
            gripper.pregrasp(goal.gripper_command)

        # Go to pick/place pose.
        rospy.loginfo("Go to pick/place pose.")
        target_pose = routines.effector_target_pose(goal.pose,
                                                    (0, 0, goal.grasp_offset))
        routines.publish_marker(target_pose,
                                "pick_pose" if goal.pick else "place_pose")
        (_, _, feedback.current_pose) \
            = routines.go_to_pose_goal(goal.robot_name, target_pose,
                                       goal.speed_slow)
        self._server.publish_feedback(feedback)

        # Grasp or release
        if goal.pick:
            gripper_success = gripper.grasp(goal.gripper_command)
            rospy.loginfo("Pick {}".format("succeeded." if gripper_success else
                                           "failed."))
        else:
            gripper_success = gripper.release(goal.gripper_command)

        # Go back to approach pose.
        result = amsg.pickOrPlaceResult()
        if goal.liftup_after:
            rospy.loginfo("Go back to approach pose.")
            (result.success, _, _) \
                = routines.go_to_pose_goal(goal.robot_name,
                                           routines.effector_target_pose(
                                               goal.pose,
                                               (0, 0, goal.approach_offset)),
                                           goal.speed_fast)

        result.success = result.success and gripper_success
        self._server.set_succeeded(result)
