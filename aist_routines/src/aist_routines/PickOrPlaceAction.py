import rospy
import actionlib
from geometry_msgs import msg as gmsg
from aist_routines import msg as amsg
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
        self._server.register_preempt_callback(self._preempt_callback)
        self._server.start()
        self._client = actionlib.SimpleActionClient("pickOrPlace",
                                                    amsg.pickOrPlaceAction)
        self._client.wait_for_server()

    def shutdown(self):
        self._server.__del__()

    def execute(self, robot_name, pose_stamped, pick, grasp_offset,
                approach_offset, liftup_after, speed_fast, speed_slow,
                wait=True, feedback_cb=None):
        goal = amsg.pickOrPlaceGoal()
        goal.robot_name      = robot_name
        goal.pose            = pose_stamped
        goal.pick            = pick
        goal.grasp_offset    = gmsg.Vector3(*grasp_offset)
        goal.approach_offset = gmsg.Vector3(*approach_offset)
        goal.liftup_after    = liftup_after
        goal.speed_fast      = speed_fast
        goal.speed_slow      = speed_slow
        self._client.send_goal(goal, feedback_cb=feedback_cb)
        if wait:
            return self.wait_for_result()
        else:
            return amsg.pickOrPlaceResult.EXECUTING

    def wait_for_result(self):
        self._client.wait_for_result()
        return self._client.get_result().result

    def cancel(self):
        self._client.cancel_goal()

    def _execute_cb(self, goal):
        rospy.loginfo("*** Do {} ***".format("picking" if goal.pick else
                                             "placing"))
        routines = self._routines
        gripper  = routines.gripper(goal.robot_name)
        feedback = amsg.pickOrPlaceFeedback()
        result   = amsg.pickOrPlaceResult()

        # Move to preparation pose.
        rospy.loginfo("--- Go to approach pose. ---")
        feedback.state = amsg.pickOrPlaceFeedback.MOVING
        self._server.publish_feedback(feedback)
        (success, _, _) \
            = routines.go_to_pose_goal(goal.robot_name,
                                       routines.effector_target_pose(
                                           goal.pose,
                                           (goal.approach_offset.x,
                                            goal.approach_offset.y,
                                            goal.approach_offset.z)),
                                       goal.speed_fast if goal.pick else
                                       goal.speed_slow)
        if not success:
            result.result = amsg.pickOrPlaceResult.MOVE_FAILURE
            self._server.set_succeeded(result)
            return

        # Pregrasp
        if goal.pick:
            rospy.loginfo("--- Pregrasp. ---")
            gripper.pregrasp()

        # Approach pick/place pose.
        rospy.loginfo("--- Go to {} pose. ---"
                      .format("pick" if goal.pick else "place"))
        feedback.state = amsg.pickOrPlaceFeedback.APPROACHING
        self._server.publish_feedback(feedback)
        target_pose = routines.effector_target_pose(goal.pose,
                                                    (goal.grasp_offset.x,
                                                     goal.grasp_offset.y,
                                                     goal.grasp_offset.z))
        routines.publish_marker(target_pose,
                                "pick_pose" if goal.pick else "place_pose")
        (success, _, _) = routines.go_to_pose_goal(goal.robot_name, target_pose,
                                                   goal.speed_slow)
        if not success:
            result.result = amsg.pickOrPlaceResult.APPROACH_FAILURE
            self._server.set_succeeded(result)
            return

        # Grasp or release
        feedback.state = amsg.pickOrPlaceFeedback.GRASPING_OR_RELEASING
        self._server.publish_feedback(feedback)
        if goal.pick:
            success = gripper.grasp()
            if success:
                rospy.loginfo("--- Pick succeeded. ---")
            else:
                rospy.logwarn("--- Pick failed. ---")
        else:
            success = gripper.release()
            if success:
                rospy.loginfo("--- Place succeeded. ---")
            else:
                rospy.logwarn("--- Place failed. ---")

        # Depart from pick/place pose.
        if goal.liftup_after:
            rospy.loginfo("--- Go back to approach pose. ---")
            feedback.state = amsg.pickOrPlaceFeedback.DEPARTING
            self._server.publish_feedback(feedback)
            (success, _, _) \
                = routines.go_to_pose_goal(goal.robot_name,
                                           routines.effector_target_pose(
                                               goal.pose,
                                               (goal.approach_offset.x,
                                                goal.approach_offset.y,
                                                goal.approach_offset.z)),
                                           goal.speed_slow if goal.pick else
                                           goal.speed_fast,
                                           move_lin=True)
            if not success:
                result.result = amsg.pickOrPlaceResult.DEPARTURE_FAILURE
                self._server.set_succeeded(result)
                return

            if goal.pick and hasattr(gripper, "suctioned"):
                gripper.postgrasp()
                success = gripper.suctioned
                if success:
                    rospy.loginfo("--- Suction succeeded. ---")
                else:
                    rospy.logwarn("--- Suction failed. ---")
                    gripper.release()

        result.result = amsg.pickOrPlaceResult.SUCCESS if success else \
                        amsg.pickOrPlaceResult.PICK_FAILURE
        self._server.set_succeeded(result)

    def _preempt_callback(self):
        routines.stop(self._server.current_goal.get_goal().robot_name)
        self._server.set_preempted()
