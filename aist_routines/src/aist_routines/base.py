import sys
import copy
import rospy
from math import pi, radians, degrees

from tf import TransformListener, transformations as tfs
import moveit_commander
from moveit_commander.conversions import pose_to_list

from geometry_msgs import msg as gmsg

from GripperClient      import GripperClient, Robotiq85Gripper, \
                               SuctionGripper, PrecisionGripper
from CameraClient       import CameraClient, PhoXiCamera, RealsenseCamera
from GraspabilityClient import GraspabilityClient
from MarkerPublisher    import MarkerPublisher
from PickOrPlaceAction  import PickOrPlaceAction

######################################################################
#  class AISTBaseRoutines                                            #
######################################################################
class AISTBaseRoutines(object):
    def __init__(self):
        super(AISTBaseRoutines, self).__init__()

        rospy.init_node("aist_routines", anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        self.listener = TransformListener()
        rospy.sleep(1.0)        # Necessary for listner spinning up

        # Grippers and cameras
        self._grippers = {
            # "a_bot": self._create_device(Robotiq85Gripper("a_bot_")),
            "a_bot": self._create_device(PrecisionGripper("a_bot_")),
            "b_bot": self._create_device(SuctionGripper("b_bot_single_")),
            "c_bot": self._create_device(Robotiq85Gripper("c_bot_")),
            #"d_bot": self._create_device(SuctionGripper("d_bot_dual_")),
            "d_bot": self._create_device(Robotiq85Gripper("d_bot_")),
        }
        self._cameras = {
            "a_phoxi_m_camera":
                self._create_device(PhoXiCamera("a_phoxi_m_camera")),
            "a_bot_camera":
                self._create_device(RealsenseCamera("a_bot_camera")),
        }

        # Grippers and cameras
        # if rospy.get_param("use_real_robot", False):
        #     self._grippers = { }
        #     _dict = rospy.get_param("/aist_kitting/grippers/no_real_robot")
        #     for key in _dict.keys():
        #         self._grippers[key] = globals()[_dict[key]['class']](
        #                                                 **_dict[key]['kwarg'])
        #     self._cameras = { }
        #     _dict = rospy.get_param("/aist_kitting/cameras/no_real_robot")
        #     for key in _dict.keys():
        #         self._cameras[key] = globals()[_dict[key]['class']](
        #                                                 **_dict[key]['kwarg'])
        # else:
        #     self._grippers = { }
        #     _dict = rospy.get_param("/aist_kitting/grippers/real_robot")
        #     for key in _dict.keys():
        #         self._grippers[key] = globals()[_dict[key]['class']](
        #                                                 **_dict[key]['kwarg'])
        #     self._cameras = { }
        #     _dict = rospy.get_param("/aist_kitting/cameras/real_robot")
        #     for key in _dict.keys():
        #         self._cameras[key] = globals()[_dict[key]['class']](
        #                                                 **_dict[key]['kwarg'])

        self._markerPublisher    = MarkerPublisher()
        self._graspabilityClient = GraspabilityClient()
        self._pickOrPlaceAction  = PickOrPlaceAction(self)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self._pickOrPlaceAction.shutdown()
        rospy.signal_shutdown("AISTBaseRoutines() completed.")
        return False  # Do not forward exceptions

    # Basic motion stuffs
    def go_to_named_pose(self, named_pose, group_name):
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_named_target(named_pose)
        group.set_max_velocity_scaling_factor(1.0)
        success = group.go(wait=True)
        group.clear_pose_targets()
        return success

    def go_to_frame(self, robot_name, target_frame, offset=(0, 0, 0),
                    speed=1.0, high_precision=False, end_effector_link="",
                    move_lin=True):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = gmsg.Pose(gmsg.Point(0, 0, 0),
                                                gmsg.Quaternion(0, 0, 0, 1))
        return self.go_to_pose_goal(robot_name,
                                    self.effector_target_pose(target_pose,
                                                              offset),
                                    speed, high_precision, end_effector_link,
                                    move_lin)

    def go_to_pose_goal(self, robot_name, target_pose, speed=1.0,
                        high_precision=False, end_effector_link="",
                        move_lin=True):
        # rospy.loginfo("move to " + self.format_pose(target_pose))
        self.publish_marker(target_pose, "pose")

        if end_effector_link == "":
            end_effector_link = self._grippers[robot_name].tip_link

        group = moveit_commander.MoveGroupCommander(robot_name)
        group.set_end_effector_link(end_effector_link)
        group.set_max_velocity_scaling_factor(self._clamp(speed, 0.0, 1.0))

        if high_precision:
            goal_tolerance = group.get_goal_tolerance()
            planning_time  = group.get_planning_time()
            group.set_goal_tolerance(.000001)
            group.set_planning_time(10)

        if move_lin:
            try:
                self.listener.waitForTransform(group.get_planning_frame(),
                                               target_pose.header.frame_id,
                                               rospy.Time.now(),
                                               rospy.Duration(10))
                pose_world = self.listener.transformPose(
                                group.get_planning_frame(), target_pose).pose
            except Exception as e:
                rospy.logerr("AISTBaseRoutines.go_to_pose_goal(): {}"
                             .format(e))
                return (False, False, group.get_current_pose())
            waypoints = []
            waypoints.append(pose_world)
            (plan, fraction) = group.compute_cartesian_path(waypoints,
                                                            0.0005,  # eef_step
                                                            0.0) # jump_threshold
            if fraction < 0.995:
                rospy.logwarn("Computed only {}% of the total cartesian path."
                              .format(fraction*100))
                group.clear_pose_targets()
                return (False, False, group.get_current_pose())

            rospy.loginfo("Execute plan with {}% computed cartesian path."
                          .format(fraction*100))
            robots  = moveit_commander.RobotCommander()
            plan    = group.retime_trajectory(robots.get_current_state(),
                                              plan, speed)
            success = group.execute(plan, wait=True)
        else:
            group.set_pose_target(target_pose)
            success = group.go(wait=True)

        rospy.loginfo("go_to_pose_goal() {}"
                      .format("succeeded." if success else "failed."))

        if high_precision:
            group.set_goal_tolerance(goal_tolerance[1])
            group.set_planning_time(planning_time)

        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        current_pose = group.get_current_pose()
        is_all_close = self._all_close(target_pose, current_pose, 0.01)
        # rospy.loginfo("reached " + self.format_pose(current_pose))
        return (success, is_all_close, current_pose)

    def get_current_pose(self, robot_name):
        group = moveit_commander.MoveGroupCommander(robot_name)
        return group.get_current_pose()

    # Gripper stuffs
    def gripper(self, robot_name):
        return self._grippers[robot_name]

    def pregrasp(self, robot_name, command=""):
        return self._grippers[robot_name].pregrasp(command)

    def grasp(self, robot_name, command=""):
        return self._grippers[robot_name].grasp(command)

    def release(self, robot_name, command=""):
        return self._grippers[robot_name].release(command)

    # Camera stuffs
    def camera(self, camera_name):
        return self._cameras[camera_name]

    def continuous_shot(self, camera_name, enable):
        return self._cameras[camera_name].continuous_shot(enable)

    def trigger_frame(self, camera_name):
        return self._cameras[camera_name].trigger_frame()

    # Marker stuffs
    def delete_all_markers(self):
        self._markerPublisher.delete_all()

    def publish_marker(self, pose_stamped, marker_type, text="", lifetime=15):
        return self._markerPublisher.add(pose_stamped, marker_type,
                                         text, lifetime)

    # Graspability stuffs
    def create_background_image(self, camera_name):
        camera = self._cameras[camera_name]
        camera.continuous_shot(True)
        success = self._graspabilityClient.create_background_image(
                        camera.depth_topic)
        camera.continuous_shot(False)
        return success

    def create_mask_image(self, camera_name, nbins):
        camera = self._cameras[camera_name]
        camera.continuous_shot(True)
        success = self._graspabilityClient.create_mask_image(
                        camera.image_topic, nbins)
        camera.continuous_shot(False)
        return success

    def search_graspability(self, robot_name, camera_name, part_id, bin_id,
                            use_normals=True, marker_lifetime=60):
        self.delete_all_markers()
        gripper = self._grippers[robot_name]
        camera  = self._cameras[camera_name]
        camera.continuous_shot(True)
        (poses, gscore, success) = \
            self._graspabilityClient.search(camera.camera_info_topic,
                                            camera.depth_topic,
                                            camera.normal_topic if use_normals \
                                            else "",
                                            gripper.type, part_id, bin_id)
        camera.continuous_shot(False)
        if success:
            for i, pose in enumerate(poses):
                self.publish_marker(pose, "graspability",
                                    "{}[{:.3f}]".format(i, gscore[i]),
                                    lifetime=marker_lifetime)
                rospy.loginfo("graspability: {}[{:.3f}]".format(i, gscore[i]))

        return (poses, gscore, success)

    def graspability_send_goal(self, robot_name, camera_name, part_id, bin_id,
                               use_normals=True):
        self.delete_all_markers()
        gripper = self._grippers[robot_name]
        camera  = self._cameras[camera_name]
        camera.continuous_shot(True)
        self._graspabilityClient.send_goal(camera.camera_info_topic,
                                           camera.depth_topic,
                                           camera.normal_topic if use_normals \
                                           else "",
                                           gripper.type, part_id, bin_id)

    def graspability_wait_for_result(self, camera_name, marker_lifetime=60):
        (poses, gscore, success) = \
            self._graspabilityClient.wait_for_result()
        camera = self._cameras[camera_name]
        camera.continuous_shot(False)
        if success:
            for i, pose in enumerate(poses):
                self.publish_marker(pose, "graspability",
                                    "{}[{:.3f}]".format(i, gscore[i]),
                                    lifetime=marker_lifetime)
                rospy.loginfo("graspability: {}[{:.3f}]".format(i, gscore[i]))

        return (poses, gscore, success)

    def graspability_cancel_goal(self):
        self._graspabilityClient.cancel_goal()

    # Pick and place action stuffs
    def pick(self, robot_name, target_pose,
             grasp_offset=0.0, gripper_command="",
             speed_fast=1.0, speed_slow=0.04, approach_offset=0.10,
             liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        return self._pickOrPlaceAction.execute(
            robot_name, target_pose, True, gripper_command,
            grasp_offset, approach_offset, liftup_after,
            speed_fast, speed_slow, acc_fast, acc_slow)

    def place(self, robot_name, target_pose,
              place_offset=0.0, gripper_command="",
              speed_fast=1.0, speed_slow=0.04, approach_offset=0.05,
              liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        return self._pickOrPlaceAction.execute(
            robot_name, target_pose, False, gripper_command,
            place_offset, approach_offset, liftup_after,
            speed_fast, speed_slow, acc_fast, acc_slow)

    def pick_at_frame(self, robot_name, target_frame, offset=(0, 0, 0),
                      grasp_offset=0.0, gripper_command="",
                      speed_fast=1.0, speed_slow=0.04, approach_offset=0.05,
                      liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = gmsg.Pose(gmsg.Point(*offset),
                                                gmsg.Quaternion(0, 0, 0, 1))
        return self.pick(robot_name, target_pose,
                         grasp_offset, gripper_command,
                         speed_fast, speed_slow, approach_offset,
                         liftup_after, acc_fast, acc_slow)

    def place_at_frame(self, robot_name, target_frame, offset=(0, 0, 0),
                       place_offset=0.0, gripper_command="",
                       speed_fast=1.0, speed_slow=0.04, approach_offset=0.05,
                       liftup_after=True, acc_fast=1.0, acc_slow=0.5):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = gmsg.Pose(gmsg.Point(*offset),
                                                gmsg.Quaternion(0, 0, 0, 1))
        return self.place(robot_name, target_pose,
                          place_offset, gripper_command,
                          speed_fast, speed_slow, approach_offset,
                          liftup_after, acc_fast, acc_slow)

    # Utility functions
    def format_pose(self, poseStamped):
        pose = self.listener.transformPose("workspace_center",
                                            poseStamped).pose
        rpy  = map(degrees, tfs.euler_from_quaternion([pose.orientation.x,
                                                       pose.orientation.y,
                                                       pose.orientation.z,
                                                       pose.orientation.w]))
        return "[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]".format(
            pose.position.x, pose.position.y, pose.position.z, *rpy)

    def effector_target_pose(self, target_pose, offset):
        T = tfs.concatenate_matrices(
                self.listener.fromTranslationRotation(
                    (target_pose.pose.position.x,
                     target_pose.pose.position.y,
                     target_pose.pose.position.z),
                    (target_pose.pose.orientation.x,
                     target_pose.pose.orientation.y,
                     target_pose.pose.orientation.z,
                     target_pose.pose.orientation.w)),
                self.listener.fromTranslationRotation(
                    offset,
                    tfs.quaternion_from_euler(0, radians(90), 0)))
        pose = gmsg.PoseStamped()
        pose.header = target_pose.header
        pose.pose = gmsg.Pose(gmsg.Point(*tfs.translation_from_matrix(T)),
                              gmsg.Quaternion(*tfs.quaternion_from_matrix(T)))
        return pose

    # Private functions
    def _create_device(self, dev):
        if rospy.get_param("use_real_robot", False):
            return dev
        else:
            return dev.base()

    def _all_close(self, goal, actual, tolerance):
        goal_list   = pose_to_list(goal.pose)
        actual_list = pose_to_list(actual.pose)
        for i in range(len(goal_list)):
            if abs(actual_list[i] - goal_list[i]) > tolerance:
                return False
        return True

    def _clamp(self, x, min_x, max_x):
        return min(max(min_x, x), max_x)
