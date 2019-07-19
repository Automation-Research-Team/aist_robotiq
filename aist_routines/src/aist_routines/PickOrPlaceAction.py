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
        self._server   = actionlib.SimpleActionServer(
                                "pickOrPlace", amsg.pickOrPlaceAction,
                                execute_cb=self._execute_cb, auto_start=False)
        self._server.start()
        self._client = actionlib.SimpleActionClient("pickOrPlace",
                                                    amsg.pickOrPlaceAction)
        self._client.wait_for_server()

    def shutdown(self):
        self._server.__del__()

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
        self._client.send_goal(goal, feedback_cb=self._feedback_cb)
        self._client.wait_for_result()
        return self._client.get_result().result

    def _feedback_cb(self, feedback):
        pass

    def _execute_cb(self, goal):
        rospy.loginfo("*** Do {} ***".format("picking" if goal.pick else
                                             "placing"))
        routines = self._routines
        gripper  = routines.gripper(goal.robot_name)
        feedback = amsg.pickOrPlaceFeedback()

        # Move to preparation pose.
        result = amsg.pickOrPlaceResult()
        rospy.loginfo("--- Go to approach pose. ---")
        (success, _, feedback.current_pose) \
            = routines.go_to_pose_goal(goal.robot_name,
                                       routines.effector_target_pose(
                                           goal.pose,
                                           (0, 0, goal.approach_offset)),
                                       goal.speed_fast if goal.pick else
                                       goal.speed_slow,
                                       move_lin=True)
        self._server.publish_feedback(feedback)
        if not success:
            result.result = amsg.pickOrPlaceResult.MOVE_FAILURE
            self._server.set_succeeded(result)
            return

        # Pregrasp
        if goal.pick:
            rospy.loginfo("Pregrasp.")
            gripper.pregrasp(goal.gripper_command)

        # Approach pick/place pose.
        rospy.loginfo("--- Go to {} pose. ---"
                      .format("pick" if goal.pick else "place"))
        target_pose = routines.effector_target_pose(goal.pose,
                                                    (0, 0, goal.grasp_offset))
        routines.publish_marker(target_pose,
                                "pick_pose" if goal.pick else "place_pose")
        (success, _, feedback.current_pose) \
            = routines.go_to_pose_goal(goal.robot_name, target_pose,
                                       goal.speed_slow, move_lin=True)
        self._server.publish_feedback(feedback)
        if not success:
            result.result = amsg.pickOrPlaceResult.APPROACH_FAILURE
            self._server.set_succeeded(result)
            return

        # Grasp or release
        if goal.pick:
            success = gripper.grasp(goal.gripper_command)
            rospy.loginfo("--- Pick {}. ---".format("succeeded" if success else
                                                    "failed"))
        else:
            success = gripper.release(goal.gripper_command)
            rospy.loginfo("--- Place {}. ---".format("succeeded" if success else
                                                     "failed"))

        # Depart from pick/place pose.
        if goal.liftup_after:
            rospy.loginfo("--- Go back to approach pose. ---")
            (success, _, feedback.current_pose) \
                = routines.go_to_pose_goal(goal.robot_name,
                                           routines.effector_target_pose(
                                               goal.pose,
                                               (0, 0, goal.approach_offset)),
                                           goal.speed_slow if goal.pick else
                                           goal.speed_fast,
                                           move_lin=True)
            self._server.publish_feedback(feedback)
            if not success:
                result.result = amsg.pickOrPlaceResult.DEPARTURE_FAILURE
                self._server.set_succeeded(result)
                return
            if goal.pick and hasattr(gripper, "suctioned"):
                success = gripper.suctioned

        result.result = amsg.pickOrPlaceResult.SUCCESS if success else \
                        amsg.pickOrPlaceResult.PICK_FAILURE
        self._server.set_succeeded(result)
