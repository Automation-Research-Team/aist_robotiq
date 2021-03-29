import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs      import msg as gmsg
from aist_routines      import msg as amsg
from tf                 import transformations as tfs

######################################################################
#  class PickOrPlaceAction                                           #
######################################################################
class PickOrPlaceAction(object):
    def __init__(self, routines):
        super(PickOrPlaceAction, self).__init__()

        self._routines = routines
        self._server   = actionlib.SimpleActionServer("pickOrPlace",
                                                      amsg.pickOrPlaceAction,
                                                      self._execute_cb, False)
        self._server.register_preempt_callback(self._preempt_callback)
        self._server.start()
        self._client = actionlib.SimpleActionClient("pickOrPlace",
                                                    amsg.pickOrPlaceAction)
        self._client.wait_for_server()

    # Client stuffs
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
            return None

    def wait_for_result(self, timeout=rospy.Duration(0)):
        if self._client.wait_for_result(timeout):
            return self._client.get_result().result
        else:
            return None

    def cancel(self):
        if self._client.get_state() in ( GoalStatus.PENDING,
                                         GoalStatus.ACTIVE ):
            self._client.cancel_goal()

    # Server stuffs
    def shutdown(self):
        self._server.__del__()

    def _execute_cb(self, goal):
        rospy.loginfo("*** Do %s ***", "picking" if goal.pick else "placing")
        routines = self._routines
        gripper  = routines.gripper(goal.robot_name)
        result   = amsg.pickOrPlaceResult()

        # Go to approach pose.
        rospy.loginfo("--- Go to approach pose. ---")
        if not self._is_active(amsg.pickOrPlaceFeedback.MOVING):
            return
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
            self._server.set_aborted(result, "Failed to go to approach pose")
            return

        # Approach pick/place pose.
        rospy.loginfo("--- Go to %s pose. ---",
                      "pick" if goal.pick else "place")
        if not self._is_active(amsg.pickOrPlaceFeedback.APPROACHING):
            return
        if goal.pick:
            gripper.pregrasp(False)             # Pregrasp
        target_pose = routines.effector_target_pose(goal.pose,
                                                    (goal.grasp_offset.x,
                                                     goal.grasp_offset.y,
                                                     goal.grasp_offset.z))
        routines.add_marker("pick_pose" if goal.pick else "place_pose",
                            target_pose)
        routines.publish_marker()
        (success, _, _) = routines.go_to_pose_goal(goal.robot_name, target_pose,
                                                   goal.speed_slow)
        if not success:
            result.result = amsg.pickOrPlaceResult.APPROACH_FAILURE
            self._server.set_aborted(result, "Failed to approach target")
            return

        # Grasp/release at pick/place pose.
        if not self._is_active(amsg.pickOrPlaceFeedback.GRASPING_OR_RELEASING):
            return
        if goal.pick:
            gripper.wait()                      # Wait for pregrasp completed
            gripper.grasp()
        else:
            gripper.release()

        # Go back to approach pose.
        if goal.liftup_after:
            rospy.loginfo("--- Go back to approach pose. ---")

            if not self._is_active(amsg.pickOrPlaceFeedback.DEPARTING):
                return
            if goal.pick:
                gripper.postgrasp(False)
            (success, _, _) \
                = routines.go_to_pose_goal(goal.robot_name,
                                           routines.effector_target_pose(
                                               goal.pose,
                                               (goal.approach_offset.x,
                                                goal.approach_offset.y,
                                                goal.approach_offset.z)),
                                           goal.speed_slow if goal.pick else
                                           goal.speed_fast)
            if not success:
                result.result = amsg.pickOrPlaceResult.DEPARTURE_FAILURE
                self._server.set_aborted(result, "Failed to depart from target")
                return
            if goal.pick:
                success = gripper.wait()  # Wait for postgrasp completed
                if success:
                    rospy.loginfo("--- Pick succeeded. ---")
                else:
                    rospy.logwarn("--- Pick failed. ---")

        if success:
            result.result = amsg.pickOrPlaceResult.SUCCESS
            self._server.set_succeeded(result, "Succeeded")
        else:
            result.result = amsg.pickOrPlaceResult.GRASP_FAILURE
            self._server.set_aborted(result, "Failed to grasp")

    def _preempt_callback(self):
        robot_name = self._server.current_goal.get_goal().robot_name
        self._routines.stop(robot_name)
        self._routines.gripper(robot_name).cancel()
        self._server.set_preempted()

    def _is_active(self, state):
        if self._server.is_active():
            self._server.publish_feedback(amsg.pickOrPlaceFeedback(state=state))
            return True
        return False
