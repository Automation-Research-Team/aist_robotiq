#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Toshio UESHIBA

import sys
import copy
import rospy
from math import pi, radians, degrees

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
def is_program_running(topic_namespace = ""):
    """Checks if a program is running on the UR"""
    msg = rospy.wait_for_message(topic_namespace + "/ur_driver/robot_mode_state", ur_modern_driver.msg.RobotModeDataMsg)
    if msg:
        return msg.is_program_running
    else:
        rospy.logerr("No message received from the robot. Is everything running? Is the namespace entered correctly with a leading slash?")
        return False

def wait_for_UR_program(topic_namespace="",
                        timeout_duration=rospy.Duration.from_sec(20.0)):
    rospy.logdebug("Waiting for UR program to finish.")
    # Only run this after sending custom URScripts and not the regular
    # motion commands, or this call will not terminate before the timeout.
    rospy.sleep(1.0)
    t_start = rospy.Time.now()
    time_passed = rospy.Time.now() - t_start
    while is_program_running(topic_namespace):
        rospy.sleep(.05)
        time_passed = rospy.Time.now() - t_start
        if time_passed > timeout_duration:
            rospy.loginfo("Timeout reached.")
        return False
    rospy.logdebug("UR Program has terminated.")
    return True

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
#  class AISTBaseRoutines                                            #
######################################################################
class AISTBaseRoutines(object):
    def __init__(self):
        super(AISTBaseRoutines, self).__init__()
        rospy.init_node("aist_routines", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        if rospy.get_param("use_real_robot", False):
            self._grippers = {
                # "a_bot": Robotiq85Gripper("a_bot_"),
                "a_bot": PrecisionGripper("a_bot_"),
                "b_bot": SuctionGripper("b_bot_single_"),
                "c_bot": Robotiq85Gripper("c_bot_"),
                "d_bot": SuctionGripper("d_bot_dual_")
            }
            self._cameras = {
                "a_phoxi_m_camera": PhoXiCamera("a_phoxi_m_camera"),
                "a_bot_camera":     RealsenseCamera("a_bot_camera"),
            }
        else:
            self._grippers = {
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
            self._cameras = {
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

        # Marker stuffs
        self._markerPublisher  = MarkerPublisher()

        # Graspability stuffs
        self._graspabilityClient = GraspabilityClient()

        # Action servers
        self._pickOrPlaceAction = actionlib.SimpleActionServer(
                                        "aist_skills/pickOrPlace",
                                        aist_msgs.msg.pickOrPlaceAction,
                                        execute_cb=self._pick_or_place_cb,
                                        auto_start=False)
        self._pickOrPlaceAction.start()

        # Action clients
        self._pickOrPlaceClient = actionlib.SimpleActionClient(
                                        '/aist_skills/pickOrPlace',
                                        aist_msgs.msg.pickOrPlaceAction)
        self._pickOrPlaceClient.wait_for_server()

        self._listener = TransformListener()

    def cycle_through_calibration_poses(self, poses, robot_name,
                                        speed=0.3, move_lin=False,
                                        go_home=True, end_effector_link=""):
        home_pose = "home"
        if "screw" in end_effector_link:
            home_pose = "screw_ready"

        for pose in poses:
            rospy.loginfo("============ Press `Enter` to move "
                          + robot_name + " to " + pose.header.frame_id)
            self.publish_marker(pose, "place_pose")
            raw_input()
            if go_home:
                self.go_to_named_pose(robot_name, home_pose)
            if rospy.is_shutdown():
                break
            else:
                self.go_to_pose_goal(robot_name, pose,speed=speed,
                                     end_effector_link=end_effector_link,
                                     move_lin=move_lin)

            rospy.loginfo("============ Press `Enter` to proceed ")
            raw_input()

        if go_home:
            rospy.loginfo("Moving all robots home again.")
            self.go_to_named_pose(robot_name, "home")
        return

    # Basic motion stuffs
    def go_to_named_pose(self, named_pose, group_name):
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_named_target(named_pose)
        group.set_max_velocity_scaling_factor(1.0)
        success = group.go(wait=True)
        group.clear_pose_targets()
        return success

    def go_to_pose_goal(self, robot_name, target_pose, speed=1.0,
                        high_precision=False, end_effector_link="",
                        move_lin=False):
        rospy.loginfo("move to " + self._format_pose(target_pose))
        self._markerPublisher.add(target_pose, "pose")

        if end_effector_link == "":
            end_effector_link = self._grippers[robot_name].tip_link

        group = moveit_commander.MoveGroupCommander(robot_name)
        group.set_end_effector_link(end_effector_link)
        group.set_pose_target(target_pose)
        group.set_max_velocity_scaling_factor(clamp(speed, 0.0, 1.0))

        res = aist_msgs.srv.goToPoseGoalResponse()

        if move_lin:
            pose_world = self._listener.transformPose(
                                group.get_planning_frame(), target_pose).pose
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

    # def do_linear_push(self, robot_name, force, wait=True, direction="Z+", max_approach_distance=0.1, forward_speed=0.0):
    #     if not self.use_real_robot:
    #         return True
    #     # Directly calls the UR service rather than the action of the skill_server
    #     req = o2as_msgs.srv.sendScriptToURRequest()
    #     req.robot_name = robot_name
    #     req.max_force = force
    #     req.force_direction = direction
    #     req.max_approach_distance = max_approach_distance
    #     req.forward_speed = forward_speed
    #     req.program_id = "linear_push"
    #     res = self.urscript.call(req)
    #     if wait:
    #         rospy.sleep(2.0)    # This program seems to take some time
    #         wait_for_UR_program("/" + robot_name +"_controller", rospy.Duration.from_sec(30.0))
    #     return res.success

    # Gripper stuffs
    @property
    def gripper(self, robot_name):
        return self._grippers[robot_name]

    def pregrasp(self, robot_name, command=""):
        return self.gripper(robot_name).pregrasp(command)

    def grasp(self, robot_name, command=""):
        return self.gripper(robot_name).grasp(command)

    def release(self, robot_name, command=""):
        return self.gripper(robot_name).release(command)

    # Camera stuffs
    @property
    def camera(self, camera_name):
        return self._cameras[camera_name]

    def start_acquisition(self, camera_name):
        return self.camera(camera_name).start_acquisition()

    def stop_acquisition(self, camera_name):
        return self.camera(camera_name).stop_acquisition()

    # Marker stuffs
    def publish_marker(self, pose_stamped, marker_type):
        return self._markerPublisher.add(pose_stamped, marker_type)

    # Graspability stuffs
    def create_mask_image(self, camera_name, nbins):
        camera = self._cameras[camera_name]
        camera.start_acquisition()
        success = self._graspabilityClient.create_mask_image(
                        camera.image_topic, nbins)
        camera.stop_acquisition()
        return success

    def search_graspability(self, robot_name, camera_name, part_id, bin_id):
        gripper = self._grippers[robot_name]
        camera  = self._cameras[camera_name]
        camera.start_acquisition()
        (poses, rotipz, gscore, success) = \
            self._graspabilityClient.search(camera.camera_info_topic,
                                            camera.depth_topic,
                                            "", gripper.type, part_id, bin_id)
        camera.stop_acquisition()

        if success:
            for i in range(len(poses.poses)):
                pose = gmsg.PoseStamped()
                pose.header = poses.header
                pose.pose   = poses.poses[i]
                self._markerPublisher.add(pose, "graspability",
                                          "{}[{:.3f}]".format(i, gscore[i]),
                                          60)

        return (poses, rotipz, gscore, success)

    # Various actions
    def pick(self, robot_name, pose_stamped, grasp_offset=0.0,
             gripper_command="close",
             speed_fast=1.0, speed_slow=0.1, approach_offset=0.05,
             liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        return self._pick_or_place(robot_name, pose_stamped, True,
                                   gripper_command,
                                   grasp_offset, approach_offset, liftup_after,
                                   speed_fast, speed_slow, acc_fast, acc_slow)

    def place(self, robot_name, pose_stamped, grasp_offset=0.0,
              gripper_command="open",
              speed_fast=1.0, speed_slow=0.1, approach_offset=0.05,
              liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        return self._pick_or_place(robot_name, pose_stamped, False,
                                   gripper_command,
                                   grasp_offset, approach_offset, liftup_after,
                                   speed_fast, speed_slow, acc_fast, acc_slow)

    # Private functions
    def _pick_or_place(self, robot_name, pose_stamped, pick, gripper_command,
                       grasp_offset, approach_offset, liftup_after,
                       speed_fast, speed_slow, acc_fast, acc_slow):
        goal = aist_msgs.msg.pickOrPlaceGoal()
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
        self._pickOrPlaceClient.send_goal(goal)
        self._pickOrPlaceClient.wait_for_result()
        return self._pickOrPlaceClient.get_result()

    def _pick_or_place_cb(self, goal):
        gripper  = self._grippers[goal.robot_name]
        feedback = aist_msgs.msg.pickOrPlaceFeedback()

        # Go to approach pose.
        rospy.loginfo("Go to approach pose:")
        res = self.go_to_pose_goal(goal.robot_name,
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
        res = self.go_to_pose_goal(goal.robot_name, target_pose,
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
            res = self.go_to_pose_goal(goal.robot_name,
                                       self._gripper_target_pose(
                                           goal.pose, goal.approach_offset),
                                       goal.speed_fast)

        result = aist_msgs.msg.pickOrPlaceResult()
        result.success      = res.success and res_gripper
        result.current_pose = res.current_pose
        self._pickOrPlaceAction.set_succeeded(result)

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
