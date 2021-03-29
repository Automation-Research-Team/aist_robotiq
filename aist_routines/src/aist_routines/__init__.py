import sys
import copy
import collections
import rospy
import numpy as np

from math import pi, radians, degrees
from tf import TransformListener, transformations as tfs
import moveit_commander
from moveit_commander.conversions import pose_to_list

from geometry_msgs import msg as gmsg

from GripperClient     import GripperClient, VoidGripper
from CameraClient      import CameraClient
from MarkerPublisher   import MarkerPublisher
from PickOrPlaceAction import PickOrPlaceAction

######################################################################
#  global functions                                                  #
######################################################################
def paramtuples(d):
    fields = set()
    for params in d.values():
        for field in params.keys():
            fields.add(field)
    ParamTuple = collections.namedtuple('ParamTuple', ' '.join(fields))

    params = {}
    for key, param in d.items():
        params[key] = ParamTuple(**param)
    return params

######################################################################
#  class AISTBaseRoutines                                            #
######################################################################
class AISTBaseRoutines(object):
    def __init__(self):
        super(AISTBaseRoutines, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self._listener = TransformListener()
        rospy.sleep(1.0)        # Necessary for listner spinning up

        # MoveIt planning parameters
        self._reference_frame = rospy.get_param('~moveit_pose_reference_frame',
                                                'workspace_center')
        self._eef_step        = rospy.get_param('~moveit_eef_step', 0.0005)
        rospy.loginfo('reference_frame = {}, eef_step = {}'
                      .format(self._reference_frame, self._eef_step))

        # MoveIt RobotCommander
        self._cmd = moveit_commander.RobotCommander('robot_description')

        # Grippers
        d = rospy.get_param('~grippers', {})
        self._grippers = {'void_gripper':
                          VoidGripper('void_gripper',
                                      'void_gripper_base_link')}
        for gripper_name, props in d.items():
            self._grippers[gripper_name] = GripperClient.create(props['type'],
                                                                props['args'])

        # Robots
        d = rospy.get_param('~robots', {})
        self._active_grippers = {}
        for robot_name, props in d.items():
            if 'default_gripper' in props:
                self.set_gripper(robot_name, props['default_gripper'])
            else:
                self.set_gripper(robot_name, 'void_gripper')

        # Cameras
        d = rospy.get_param('~cameras', {})
        self._cameras = {}
        for camera_name, type_name in d.items():
            self._cameras[camera_name] = CameraClient.create(camera_name,
                                                             type_name)

        # Search graspabilities
        if rospy.has_param('~graspability_parameters'):
            from aist_graspability import GraspabilityClient
            self._graspability_params \
                = rospy.get_param('~graspability_parameters')
            self._graspabilityClient = GraspabilityClient()

        # Pick and place action
        if rospy.has_param('~picking_parameters'):
            self._picking_params    = rospy.get_param('~picking_parameters')
            self._pickOrPlaceAction = PickOrPlaceAction(self)
        else:
            self._pickOrPlaceAction = None

        # Marker publisher
        self._markerPublisher = MarkerPublisher()

        rospy.loginfo('AISTBaseRoutines initialized.')

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        if self._pickOrPlaceAction:
            self._pickOrPlaceAction.shutdown()
        rospy.signal_shutdown('AISTBaseRoutines() completed.')
        return False  # Do not forward exceptions

    @property
    def listener(self):
        return self._listener

    @property
    def reference_frame(self):
        return self._reference_frame

    @property
    def eef_step(self):
        return self._eef_step

    # Basic motion stuffs
    def go_to_named_pose(self, robot_name, named_pose):
        group = self._cmd.get_group(robot_name)
        try:
            group.set_named_target(named_pose)
        except moveit_commander.exception.MoveItCommanderException as e:
            rospy.logerr('AISTBaseRoutines.go_to_named_pose(): {}'
                         .format(e))
            return False
        group.set_max_velocity_scaling_factor(1.0)
        success = group.go(wait=True)
        group.clear_pose_targets()
        return success

    def go_to_frame(self, robot_name, target_frame, offset=(0, 0, 0),
                    speed=1.0, end_effector_link='',
                    high_precision=False, move_lin=True):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = gmsg.Pose(gmsg.Point(0, 0, 0),
                                                gmsg.Quaternion(0, 0, 0, 1))
        return self.go_to_pose_goal(robot_name,
                                    self.effector_target_pose(target_pose,
                                                              offset),
                                    speed, end_effector_link,
                                    high_precision, move_lin)

    def go_to_pose_goal(self, robot_name, target_pose,
                        speed=1.0, end_effector_link='',
                        high_precision=False, move_lin=True):
        self.add_marker('pose', target_pose)
        self.publish_marker()

        if move_lin:
            return self.go_along_poses(robot_name,
                                       gmsg.PoseArray(target_pose.header,
                                                      [target_pose.pose]),
                                       speed, end_effector_link, high_precision)

        if end_effector_link == '':
            end_effector_link = self.gripper(robot_name).tip_link

        group = self._cmd.get_group(robot_name)
        group.set_end_effector_link(end_effector_link)
        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))
        group.set_pose_target(target_pose)
        success      = group.go(wait=True)
        current_pose = group.get_current_pose()
        is_all_close = self._all_close(target_pose.pose,
                                       current_pose.pose, 0.01)
        return (success, is_all_close, current_pose)

    def go_along_poses(self, robot_name, poses,
                       speed=1.0, end_effector_link='', high_precision=False):
        if end_effector_link == '':
            end_effector_link = self.gripper(robot_name).tip_link

        group = self._cmd.get_group(robot_name)
        group.set_end_effector_link(end_effector_link)
        group.set_max_velocity_scaling_factor(np.clip(speed, 0.0, 1.0))

        try:
            transformed_poses = self.transform_poses_to_target_frame(
                                    poses, group.get_planning_frame()).poses
        except Exception as e:
            return (False, False, group.get_current_pose())

        if high_precision:
            goal_tolerance = group.get_goal_tolerance()
            planning_time  = group.get_planning_time()
            group.set_goal_tolerance(.000001)
            group.set_planning_time(10)

        plan, fraction = group.compute_cartesian_path(transformed_poses,
                                                      self._eef_step, 0.0)
        if fraction > 0.995:
            success = group.execute(group.retime_trajectory(
                                        self._cmd.get_current_state(),
                                        plan, speed),
                                    wait=True)
            group.stop()
            if success:
                rospy.loginfo('Executed plan with %3.1f%% computed cartesian path.',
                              100*fraction)
            else:
                rospy.logerr('Computed %3.1f%% of cartesian path but failed to execute.',
                             100*fraction)
        else:
            success = False
            rospy.logwarn('Computed only %3.1f%% of the total cartesian path.',
                          100*fraction)

        group.clear_pose_targets()

        if high_precision:
            group.set_goal_tolerance(goal_tolerance[1])
            group.set_planning_time(planning_time)

        current_pose = group.get_current_pose()
        is_all_close = self._all_close(transformed_poses[-1],
                                       current_pose.pose, 0.01)
        return (success, is_all_close, current_pose)

    def stop(self, robot_name):
        group = self._cmd.get_group(robot_name)
        group.stop()
        group.clear_pose_targets()

    def get_current_pose(self, robot_name, end_effector_link=''):
        if end_effector_link == '' and robot_name in self._grippers:
            end_effector_link = self.gripper(robot_name).tip_link
        group = self._cmd.get_group(robot_name)
        if len(end_effector_link) > 0:
            group.set_end_effector_link(end_effector_link)
        return group.get_current_pose()

    # Gripper stuffs
    def set_gripper(self, robot_name, gripper_name):
        self._active_grippers[robot_name] = self._grippers[gripper_name]

    def gripper(self, robot_name):
        return self._active_grippers[robot_name]

    def set_gripper_parameters(self, robot_name, parameters):
        self.gripper(robot_name).parameters = parameters

    def gripper_parameters(self, robot_name):
        return self.gripper(robot_name).parameters

    def pregrasp(self, robot_name):
        return self.gripper(robot_name).pregrasp()

    def grasp(self, robot_name):
        return self.gripper(robot_name).grasp()

    def release(self, robot_name):
        return self.gripper(robot_name).release()

    # Camera stuffs
    def camera(self, camera_name):
        return self._cameras[camera_name]

    def continuous_shot(self, camera_name, enable):
        return self.camera(camera_name).continuous_shot(enable)

    def trigger_frame(self, camera_name):
        return self.camera(camera_name).trigger_frame()

    # Marker stuffs
    def delete_all_markers(self):
        self._markerPublisher.delete_all()

    def add_marker(self, marker_type, pose_stamped, endpoint=None,
                   text='', lifetime=15):
        self._markerPublisher.add(marker_type, pose_stamped,
                                  endpoint, text, lifetime)

    def publish_marker(self):
        self._markerPublisher.publish()

    # Graspability stuffs
    def create_mask_image(self, camera_name, nmasks):
        self.camera(camera_name).trigger_frame()
        return self._graspabilityClient.create_mask_image(nmasks)

    def graspability_send_goal(self, robot_name, part_id, mask_id,
                               feedback_cb=None):
        self.delete_all_markers()
        params = self._graspability_params[part_id]
        self._graspabilityClient.set_parameters(params)

        # Send goal first to be ready for subscribing image,
        self._graspabilityClient.send_goal(mask_id,
                                           self.gripper(robot_name).type,
                                           feedback_cb)

    def graspability_wait_for_result(self, orientation=None, max_slant=pi/4,
                                     marker_lifetime=0):
        poses, gscores, contact_points \
            = self._graspabilityClient.wait_for_result(orientation, max_slant)

        #  We have to transform the poses to reference frame before moving
        #  because graspability poses are represented w.r.t. camera frame
        #  which will change while moving in the case of "eye on hand".
        contact_points = self._transform_points_to_reference_frame(
                                poses.header, contact_points)
        poses          = self.transform_poses_to_target_frame(poses)
        for i, pose in enumerate(poses.poses):
            self.add_marker('graspability',
                            gmsg.PoseStamped(poses.header, pose),
                            contact_points[i],
                            '{}[{:.3f}]'.format(i, gscores[i]),
                            lifetime=marker_lifetime)
        self.publish_marker()

        return poses, gscores

    def graspability_cancel_goal(self):
        self._graspabilityClient.cancel_goal()

    # Pick and place action stuffs
    def pick(self, robot_name, target_pose, part_id,
             wait=True, feedback_cb=None):
        params = self._picking_params[part_id]
        if 'gripper_name' in params:
            self.set_gripper(robot_name, params['gripper_name'])
        if 'gripper_parameters' in params:
            self.gripper(robot_name).parameters = params['gripper_parameters']
        return self._pickOrPlaceAction.execute(robot_name, target_pose, True,
                                               params['grasp_offset'],
                                               params['approach_offset'],
                                               params['liftup_after'],
                                               params['speed_fast'],
                                               params['speed_slow'],
                                               wait, feedback_cb)

    def place(self, robot_name, target_pose, part_id,
              wait=True, feedback_cb=None):
        params = self._picking_params[part_id]
        if 'gripper_name' in params:
            self.set_gripper(robot_name, params['gripper_name'])
        if 'gripper_parameters' in params:
            self.gripper(robot_name).parameters = params['gripper_parameters']
        return self._pickOrPlaceAction.execute(robot_name, target_pose, False,
                                               params['place_offset'],
                                               params['approach_offset'],
                                               params['liftup_after'],
                                               params['speed_fast'],
                                               params['speed_slow'],
                                               wait, feedback_cb)

    def pick_at_frame(self, robot_name, target_frame, part_id,
                      offset=(0.0, 0.0, 0.0), wait=True, feedback_cb=None):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = gmsg.Pose(gmsg.Point(*offset),
                                                gmsg.Quaternion(0, 0, 0, 1))
        return self.pick(robot_name, target_pose, part_id, wait, feedback_cb)

    def place_at_frame(self, robot_name, target_frame, part_id,
                       offset=(0.0, 0.0, 0.0), wait=True, feedback_cb=None):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = target_frame
        target_pose.pose            = gmsg.Pose(gmsg.Point(*offset),
                                                gmsg.Quaternion(0, 0, 0, 1))
        return self.place(robot_name, target_pose, part_id, wait, feedback_cb)

    def pick_or_place_wait_for_result(self):
        return self._pickOrPlaceAction.wait_for_result()

    def pick_or_place_cancel(self):
        return self._pickOrPlaceAction.cancel()

    # Utility functions
    def transform_pose_to_target_frame(self, pose, target_frame=''):
        poses = self.transform_poses_to_target_frame(
                    gmsg.PoseArray(pose.header, [pose.pose]), target_frame)
        return gmsg.PoseStamped(poses.header, poses.poses[0])

    def transform_poses_to_target_frame(self, poses, target_frame=''):
        if target_frame == '':
            target_frame = self._reference_frame

        try:
            self._listener.waitForTransform(target_frame,
                                            poses.header.frame_id,
                                            poses.header.stamp,
                                            rospy.Duration(10))
            mat44 = self._listener.asMatrix(target_frame, poses.header)
        except Exception as e:
            rospy.logerr('AISTBaseRoutines.transform_poses_to_target_frame(): {}'.format(e))
            raise e

        transformed_poses = gmsg.PoseArray()
        transformed_poses.header.frame_id = target_frame
        transformed_poses.header.stamp    = poses.header.stamp
        for pose in poses.poses:
            m44 = tfs.concatenate_matrices(
                        mat44,
                        self._listener.fromTranslationRotation(
                            (pose.position.x,
                             pose.position.y,
                             pose.position.z),
                            (pose.orientation.x,
                             pose.orientation.y,
                             pose.orientation.z,
                             pose.orientation.w)))
            transformed_poses.poses.append(
                gmsg.Pose(gmsg.Point(*tuple(tfs.translation_from_matrix(m44))),
                          gmsg.Quaternion(
                              *tuple(tfs.quaternion_from_matrix(m44)))))
        return transformed_poses

    def xyz_rpy(self, pose):
        transformed_pose = self.transform_pose_to_target_frame(pose).pose
        rpy = tfs.euler_from_quaternion((transformed_pose.orientation.x,
                                         transformed_pose.orientation.y,
                                         transformed_pose.orientation.z,
                                         transformed_pose.orientation.w))
        return [transformed_pose.position.x,
                transformed_pose.position.y,
                transformed_pose.position.z,
                rpy[0], rpy[1], rpy[2]]

    def format_pose(self, target_pose):
        xyzrpy = self.xyz_rpy(target_pose)
        return '[{:.4f}, {:.4f}, {:.4f}; {:.2f}, {:.2f}. {:.2f}]'.format(
            xyzrpy[0], xyzrpy[1], xyzrpy[2],
            degrees(xyzrpy[3]), degrees(xyzrpy[4]), degrees(xyzrpy[5]))

    def effector_target_pose(self, target_pose, offset):
        poses = self.effector_target_poses(
                    gmsg.PoseArray(target_pose.header, [target_pose.pose]),
                    [offset])
        return gmsg.PoseStamped(poses.header, poses.poses[0])

    def effector_target_poses(self, target_poses, offsets):
        poses = gmsg.PoseArray(target_poses.header, [])
        for target_pose, offset in zip(target_poses.poses, offsets):
            T = tfs.concatenate_matrices(
                    self._listener.fromTranslationRotation(
                        (target_pose.position.x,
                         target_pose.position.y,
                         target_pose.position.z),
                        (target_pose.orientation.x,
                         target_pose.orientation.y,
                         target_pose.orientation.z,
                         target_pose.orientation.w)),
                    self._listener.fromTranslationRotation(
                        offset,
                        tfs.quaternion_from_euler(0, radians(90), 0)))
            poses.poses.append(
                          gmsg.Pose(
                              gmsg.Point(*tfs.translation_from_matrix(T)),
                              gmsg.Quaternion(*tfs.quaternion_from_matrix(T))))
        return poses

    # Private functions
    def _create_device(self, type_name, kwargs):
        Device = globals()[type_name]
        if rospy.get_param('/use_real_robot', False):
            return Device(**kwargs)
        else:
            return Device.base(**kwargs)

    def _all_close(self, goal, actual, tolerance):
        goal_list   = pose_to_list(goal)
        actual_list = pose_to_list(actual)
        for i in range(len(goal_list)):
            if abs(actual_list[i] - goal_list[i]) > tolerance:
                return False
        return True

    def _transform_points_to_reference_frame(self, header, points):
        try:
            self._listener.waitForTransform(self._reference_frame,
                                            header.frame_id,
                                            header.stamp,
                                            rospy.Duration(10))
            mat44 = self._listener.asMatrix(self._reference_frame, header)
        except Exception as e:
            rospy.logerr('AISTBaseRoutines._transform_positions_to_target_frame(): {}'.format(e))
            raise e

        return [ gmsg.Point(*tuple(np.dot(mat44,
                                          np.array((p.x, p.y, p.z, 1.0)))[:3]))
                 for p in points ]
