#!/usr/bin/env python

import sys
import rospy
from math import pi, radians, degrees
import copy

from tf import TransformListener, transformations as tfs
import actionlib
import moveit_commander
from moveit_commander.conversions import pose_to_list

from geometry_msgs import msg as gmsg
import visualization_msgs.msg
import std_msgs.msg
import o2as_msgs.msg
import o2as_msgs.srv
import aist_msgs.msg
import aist_msgs.srv
import aist_graspability.msg
import aist_graspability.srv

from GripperClient      import GripperClient, Robotiq85Gripper, \
                               SuctionGripper, PrecisionGripper
from CameraClient       import CameraClient, PhoXiCamera, RealsenseCamera
from GraspabilityClient import GraspabilityClient
from MarkerPublisher    import MarkerPublisher

######################################################################
#  global fucntions                                                  #
######################################################################
def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is gmsg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is gmsg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

def clamp(x, min_x, max_x):
    return min(max(min_x, x), max_x)

######################################################################
#  class SkillServer                                                 #
######################################################################
class SkillServer(object):
    marker_props = {     #axes    scale                  color(RGBA)
        "pose":          (True,  (0.006, 0.020, 0.020), (0.0, 1.0, 0.0, 0.8)),
        "pick_pose":     (True,  (0.006, 0.020, 0.020), (1.0, 0.0, 1.0, 0.8)),
        "place_pose":    (True,  (0.006, 0.020, 0.020), (0.0, 1.0, 1.0, 0.8)),
        "graspability":  (True,  (0.004, 0.004, 0.004), (1.0, 1.0, 0.0, 0.8)),
        "":              (False, (0.004, 0.004, 0.004), (0.0, 1.0, 0.0, 0.8)),
        }

    def __init__(self):
        super(SkillServer, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)

        if rospy.get_param("use_real_robot", False):
            self.grippers = {
                # "a_bot": Robotiq85Gripper("a_bot_"),
                "a_bot": PrecisionGripper("a_bot_"),
                "b_bot": SuctionGripper("b_bot_single_"),
                "c_bot": Robotiq85Gripper("c_bot_"),
                "d_bot": SuctionGripper("d_bot_dual_")
            }
            self.cameras = {
                "a_phoxi_m_camera": PhoXiCamera("a_phoxi_m_camera"),
                "a_bot_camera":     RealsenseCamera("a_bot_camera"),
            }
        else:
            self.grippers = {
                # "a_bot": GripperClient("a_bot_robotiq_85_gripper",
                #                        "two-finger",
                #                        "a_bot_robotiq_85_base_link",
                #                        "a_bot_robotiq_85_tip_link"),
                "a_bot": GripperClient("a_bot_gripper",
                                       "two-finger",
                                       "a_bot_gripper_base_link",
                                       "a_bot_gripper_tip_link"),
                "b_bot": GripperClient("b_bot_single_suction_gripper",
                                       "suction",
                                       "b_bot_single_suction_gripper_base_link",
                                       "b_bot_single_suction_gripper_pad_link"),
                "c_bot": GripperClient("c_bot_robotiq_85_gripper",
                                       "two-finer",
                                       "c_bot_robotiq_85_base_link",
                                       "c_bot_robotiq_85_tip_link"),
                "d_bot": GripperClient("d_bot_dual_suction_gripper",
                                       "suction",
                                       "d_bot_dual_suction_gripper_base_link",
                                       "d_bot_dual_suction_gripper_pad_link")
            }
            self.cameras = {
                "a_phoxi_m_camera": CameraClient(
                                        "a_phoxi_m_camera",
                                        "depth",
                                        "/a_phoxi_m_camera/camera_info",
                                        "/a_phoxi_m_camera/depth_map"),
                "a_bot_camera":     CameraClient(
                                        "a_bot_camera",
                                        "depth",
                                        "/a_bot_camera/rgb/camera_info",
                                        "/a_bot_camera/depth/points"),
            }

        # Robot motion stuffs
        self._goToNamedPoseSrv  = rospy.Service("aist_skills/goToNamedPose",
                                                o2as_msgs.srv.goToNamedPose,
                                                self.go_to_named_pose_cb)
        self._goToPoseGoalSrv   = rospy.Service("aist_skills/goToPoseGoal",
                                                aist_msgs.srv.goToPoseGoal,
                                                self.go_to_pose_goal_cb)

        # Gripper stuffs
        self._getGripperInfoSrv = rospy.Service("aist_skills/getGripperInfo",
                                                aist_msgs.srv.getGripperInfo,
                                                self.get_gripper_info_cb)
        self._commandGripperSrv = rospy.Service("aist_skills/commandGripper",
                                                aist_msgs.srv.commandGripper,
                                                self.command_gripper_cb)

        # Camera stuffs
        self._getCameraInfoSrv  = rospy.Service("aist_skills/getCameraInfo",
                                                aist_msgs.srv.getCameraInfo,
                                                self.get_gripper_info_cb)
        self._commandCameraSrv  = rospy.Service("aist_skills/commandCamera",
                                                aist_msgs.srv.commandCamera,
                                                self.command_camera_cb)

        # Marker stuffs
        self._markerPublisher  = MarkerPublisher()
        self._publishMarkerSrv = rospy.Service("aist_skills/publishMarker",
                                               o2as_msgs.srv.publishMarker,
                                               self.publish_marker_cb)

        # Graspability stuffs
        self._graspabilityClient = GraspabilityClient()
        self._createMaskImageSrv = rospy.Service("aist_skills/createMaskImage",
                                                 aist_msgs.srv.createMaskImage,
                                                 self.create_mask_image_cb)
        self._searchGraspabilitySrv \
            = rospy.Service("aist_skills/searchGraspability",
                            aist_msgs.srv.searchGraspability,
                            self.search_graspability_cb)

        # Action servers
        self._pickOrPlaceAction = actionlib.SimpleActionServer(
                                        "aist_skills/pickOrPlace",
                                        aist_msgs.msg.pickOrPlaceAction,
                                        execute_cb=self.pick_or_place_cb,
                                        auto_start=False)
        self._pickOrPlaceAction.start()

        self._listener = TransformListener()
        rospy.loginfo("aist_skills server starting up!")

    # Robot motion stuffs
    def go_to_named_pose_cb(self, req):
        group = moveit_commander.MoveGroupCommander(req.planning_group)
        group.set_named_target(req.named_pose)
        group.set_max_velocity_scaling_factor(1.0)
        success = group.go(wait=True)
        group.clear_pose_targets()
        return success

    def go_to_pose_goal_cb(self, req):
        self._markerPublisher.add(req.target_pose, "pose")
        return self._go_to_pose_goal(req.robot_name, req.target_pose,
                                     req.speed, req.high_precision,
                                     req.end_effector_link, req.move_lin)

    def _go_to_pose_goal(self, robot_name, target_pose, speed=1.0,
                         high_precision=False, end_effector_link="",
                         move_lin=True):
        rospy.loginfo("move to " + self._format_pose(target_pose))

        if end_effector_link == "":
            end_effector_link = self.grippers[robot_name].tip_link

        group = moveit_commander.MoveGroupCommander(robot_name)
        group.set_end_effector_link(end_effector_link)
        group.set_pose_target(target_pose)
        group.set_max_velocity_scaling_factor(clamp(speed, 0.0, 1.0))

        res = aist_msgs.srv.goToPoseGoalResponse()

        if move_lin:
            pose_world = self._listener.transformPose(group.get_planning_frame(),
                                                     target_pose).pose
            waypoints  = []
            waypoints.append(pose_world)
            (plan, fraction) = group.compute_cartesian_path(waypoints,
                                                            0.0005,  # eef_step
                                                            0.0) # jump_threshold
            rospy.loginfo("Compute cartesian path succeeded with " +
                          str(fraction*100) + "%")
            robots      = moveit_commander.RobotCommander()
            plan        = group.retime_trajectory(robots.get_current_state(),
                                                  plan, speed)
            res.success = group.execute(plan, wait=True)
        else:
            goal_tolerance = group.get_goal_tolerance()
            planning_time  = group.get_planning_time()
            if high_precision:
                group.set_goal_tolerance(.000001)
                group.set_planning_time(10)
            res.success = group.go(wait=True)
            if high_precision:
                group.set_goal_tolerance(goal_tolerance)
                group.set_planning_time(planning_time)

        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        res.current_pose = group.get_current_pose()
        res.all_close    = all_close(target_pose.pose, res.current_pose.pose,
                                     0.01)
        rospy.loginfo("reached " + self._format_pose(res.current_pose))
        return res

    # Gripper stuffs
    def get_gripper_info_cb(self, req):
        gripper = self.grippers[req.name]
        res     = aist_msgs.srv.getGripperInfoResponse()
        res.type = gripper.type
        res.base_link    = gripper.base_link
        res.tip_link     = gripper.tip_link
        res.success      = True
        return res

    def command_gripper_cb(self, req):
        gripper = self.grippers[req.name]
        res     = aist_msgs.srv.commandGripperResponse()
        if req.action == 0:
            res.success = gripper.pregrasp(req.command)
        elif req.action == 1:
            res.success = gripper.grasp(req.command)
        else:
            res.success = gripper.release(req.command)
        return res

    # Camera stuffs
    def get_camera_info_cb(self, req):
        camera = self.cameras[req.name]
        res    = aist_msgs.srv.getCameraInfoResponse()
        res.type       = camera.type
        res.camera_info_topic = camera.camera_info_topic
        res.image_topic       = camera.image_topic
        res.success           = True
        return res

    def command_camera_cb(self, req):
        camera = self.cameras[req.name]
        res    = aist_msgs.srv.commandCameraResponse()
        if req.acquire:
            res.success = camera.start_acquisition()
        else:
            res.success = camera.stop_acquisition()
        return res

    # Marker stuffs
    def publish_marker_cb(self, req):
        res = o2as_msgs.srv.publishMarkerResponse()
        res.result = self._markerPublisher.add(req.marker_pose,
                                               req.marker_type)
        return res

    # Graspability stuffs
    def create_mask_image_cb(self, req):
        camera = self.cameras[req.camera_name]
        camera.start_acquisition()
        res = aist_msgs.srv.createMaskImageResponse()
        res.success = self._graspabilityClient.create_mask_image(
                        camera.image_topic, req.nbins)
        camera.stop_acquisition()
        return res

    def search_graspability_cb(self, req):
        gripper = self.grippers[req.robot_name]
        camera  = self.cameras[req.camera_name]
        camera.start_acquisition()
        (poses, rotipz, gscore, success) = \
            self._graspabilityClient.search(camera.camera_info_topic,
                                            camera.image_topic, gripper.type,
                                            req.part_id, req.bin_id)
        camera.stop_acquisition()

        if success:
            for i in range(len(poses)):
                pose = gmsg.PoseStamped()
                pose.header = marker_poses.header
                pose.pose   = marker_poses.poses[i]
                self._markerPublisher.add(pose, "graspability",
                                          "{}[{:.3f}]".format(i, gscore[i]), 0)

        res = aist_msgs.srv.searchGraspabilityResponse()
        res.poses   = poses
        res.rotipz  = rotipz
        res.gscore  = gscore
        res.success = success
        return res

    # Various actions
    def pick_or_place_cb(self, goal):
        gripper  = self.grippers[goal.robot_name]
        feedback = aist_msgs.msg.pickOrPlaceFeedback()

        # Go to approach pose.
        rospy.loginfo("Go to approach pose:")
        res = self._go_to_pose_goal(goal.robot_name,
                                    self._gripper_target_pose(
                                        goal.pose, goal.approach_offset),
                                    goal.speed_fast)
        feedback.current_pose = res.current_pose
        self._pickOrPlaceAction.publish_feedback(feedback)

        # Pregrasp
        if goal.pick:
            gripper.pregrasp(goal.gripper_command)

        # Go to pick/place pose.
        rospy.loginfo("Go to pick/place pose:")
        target_pose = self._gripper_target_pose(goal.pose, goal.grasp_offset)
        self._markerPublisher.add(target_pose,
                                  "pick_pose" if goal.pick else "place_pose")
        res = self._go_to_pose_goal(goal.robot_name, target_pose,
                                    goal.speed_slow)
        feedback.current_pose = res.current_pose
        self._pickOrPlaceAction.publish_feedback(feedback)

        # Grasp or release
        if goal.pick:
            res_gripper = gripper.grasp(goal.gripper_command)
        else:
            res_gripper = gripper.release(goal.gripper_command)

        # Go back to approach pose.
        if goal.liftup_after:
            rospy.loginfo("Go back to approach pose:")
            res = self._go_to_pose_goal(goal.robot_name,
                                        self._gripper_target_pose(
                                            goal.pose, goal.approach_offset),
                                        goal.speed_fast)

        result = aist_msgs.msg.pickOrPlaceResult()
        result.success      = res.success and res_gripper
        result.current_pose = res.current_pose
        self._pickOrPlaceAction.set_succeeded(result)

    # Private functions
    def _gripper_target_pose(self, target_pose, offset):
        T = tfs.concatenate_matrices(
                self._listener.fromTranslationRotation(
                    (target_pose.pose.position.x,
                     target_pose.pose.position.y,
                     target_pose.pose.position.z),
                    (target_pose.pose.orientation.x,
                     target_pose.pose.orientation.y,
                     target_pose.pose.orientation.z,
                     target_pose.pose.orientation.w)),
                self._listener.fromTranslationRotation(
                    (0, 0, offset),
                    tfs.quaternion_from_euler(0, radians(90), 0)))
        pose = gmsg.PoseStamped()
        pose.header.frame_id = target_pose.header.frame_id
        pose.pose = gmsg.Pose(gmsg.Point(*tfs.translation_from_matrix(T)),
                              gmsg.Quaternion(*tfs.quaternion_from_matrix(T)))
        return pose

    def _format_pose(self, poseStamped):
        pose = self._listener.transformPose("workspace_center",
                                            poseStamped).pose
        rpy  = map(
            degrees,
            tfs.euler_from_quaternion([pose.orientation.w, pose.orientation.x,
                                       pose.orientation.y, pose.orientation.z]))
        return "[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]".format(
            pose.position.x, pose.position.y, pose.position.z,
            rpy[0], rpy[1], rpy[2])


######################################################################
#  main function                                                     #
######################################################################
if __name__ == "__main__":
    rospy.init_node("aist_skills", anonymous=True)
    try:
        node = SkillServer()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
