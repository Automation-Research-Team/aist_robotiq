#!/usr/bin/env python

import sys
from math import pi, radians
import copy

import numpy as np
import rospy
import geometry_msgs.msg
import visualization_msgs.msg
import tf
import tf_conversions
import actionlib
import moveit_commander

import o2as_msgs.srv
import aist_skills.msg

def quaternion_msg_to_tf(orientation):
    """Convert quaternion geometry_msgs.msg.Quaternion to tf quaternion (as numpy.ndarray).
    """

    return np.array([[orientation.x],[orientation.y],[orientation.z],[orientation.w]])

def quaternion_tf_to_msg(quaternion):
    """Convert quaternion tf quaternion (as numpy.ndarray) to geometry_msgs.msg.Quaternion.
    """

    return geometry_msgs.msg.Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

def rotatePoseByRPY(roll, pitch, yaw, inpose):
    """Return pose rotated by rpy.

    This function is converted of o2as_helper_function.h.
    """

    q_rotate = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    q = quaternion_msg_to_tf(inpose.orientation)
    q = tf.transformations.quaternion_multiply(q, q_rotate)
    inpose.orientation = quaternion_tf_to_msg(q)

    return inpose

class SkillServer(object):
    def __init__(self):
        super(SkillServer, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        self.listener = tf.TransformListener()

        # Topics to publish
        self.pubMarker_ = rospy.Publisher("visualization_marker", visualization_msgs.msg.Marker, queue_size=10)

        # Services to advertise
        self.publishMarkerService_ = rospy.Service("aist_skills/publishMarker", o2as_msgs.srv.publishMarker, self.publishMarkerCallback)

        # Actions
        self.move_lin_action = actionlib.SimpleActionServer('aist_skills/move_lin', 
                                                            aist_skills.msg.MoveLinAction,
                                                            execute_cb=self.move_lin_action_callback,
                                                            auto_start=False)
        self.move_lin_action.start()

        self.marker_id_count = 0

        rospy.loginfo("o2as_skills server starting up!")

    def publishMarkerCallback(self, req):
        rospy.loginfo("Received publishMarker callback.")
        return self.publishMarker(req.marker_pose, req.marker_type)

    def publishMarker(self, marker_pose, marker_type):
        marker = visualization_msgs.msg.Marker()
        marker.header = copy.deepcopy(marker_pose.header)
        marker.header.stamp = rospy.Time.now()
        marker.pose = copy.deepcopy(marker_pose.pose)

        marker.ns = "markers"
        marker.id = self.marker_id_count
        self.marker_id_count += 1
        marker.lifetime = rospy.Duration(10.0)
        marker.action = visualization_msgs.msg.Marker.ADD

        if marker_type == "pose":
            self.publishPoseMarker(marker_pose)
            # Add a flat sphere
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .01
            marker.scale.y = .05
            marker.scale.z = .05
            marker.color.g = 1.0
            marker.color.a = 0.8
        elif marker_type == "pick_pose":
            self.publishPoseMarker(marker_pose)
            # Add a flat sphere
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .01
            marker.scale.y = .05
            marker.scale.z = .05
            marker.color.r = 0.8
            marker.color.g = 0.4
            marker.color.a = 0.8
        elif marker_type == "place_pose":
            self.publishPoseMarker(marker_pose)
            # Add a flat sphere
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .01
            marker.scale.y = .05
            marker.scale.z = .05
            marker.color.g = 1.0
            marker.color.a = 0.8
        elif marker_type == "aist_vision_result":
            # Add a sphere
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .01
            marker.scale.y = .01
            marker.scale.z = .01
            marker.color.r = 0.8
            marker.color.g = 0.4
            marker.color.b = 0.0
            marker.color.a = 0.8
        elif marker_type == "":
            marker.type = visualization_msgs.msg.Marker.SPHERE
            marker.scale.x = .02
            marker.scale.y = .1
            marker.scale.z = .1
            marker.color.g = 1.0
            marker.color.a = 0.8
        else:
            rospy.logwarn("No useful marker message received.")
            return False

        self.pubMarker_.publish(marker)
        if self.marker_id_count > 50:
            self.marker_id_count = 0
        return True

    def publishPoseMarker(self, marker_pose):
        marker = visualization_msgs.msg.Marker()
        marker.header = copy.deepcopy(marker_pose.header)
        marker.header.stamp = rospy.Time.now()
        marker.pose = copy.deepcopy(marker_pose.pose)

        marker.ns = "markers"
        marker.id = self.marker_id_count
        self.marker_id_count += 1
        marker.lifetime = rospy.Duration(10)
        marker.action = visualization_msgs.msg.Marker.ADD

        # This draws a TF-like frame.
        marker.type = visualization_msgs.msg.Marker.ARROW
        marker.scale.x = .1
        marker.scale.y = .01
        marker.scale.z = .01
        marker.color.a = .8

        arrow_x = visualization_msgs.msg.Marker()
        arrow_y = visualization_msgs.msg.Marker()
        arrow_z = visualization_msgs.msg.Marker()
        arrow_x = copy.deepcopy(marker)
        arrow_y = copy.deepcopy(marker)
        arrow_z = copy.deepcopy(marker)
        arrow_x.id = self.marker_id_count
        self.marker_id_count += 1
        arrow_y.id = self.marker_id_count
        self.marker_id_count += 1
        arrow_z.id = self.marker_id_count
        self.marker_id_count += 1
        arrow_x.color.r = 1.0
        arrow_y.color.g = 1.0
        arrow_z.color.b = 1.0

        arrow_y.pose = rotatePoseByRPY(0, 0, pi/2, arrow_y.pose)
        arrow_z.pose = rotatePoseByRPY(0, -pi/2, 0, arrow_z.pose)

        self.pubMarker_.publish(arrow_x)
        self.pubMarker_.publish(arrow_y)
        self.pubMarker_.publish(arrow_z)

        return True

    def move_lin_action_callback(self, goal):
        pose_goal = geometry_msgs.msg.PoseStamped()
        pose_goal.header.frame_id = goal.frame_id
        pose_goal.pose.position = goal.position
        pose_goal.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(radians(goal.orientation.x), radians(goal.orientation.y), radians(goal.orientation.z)))
        goal_res = aist_skills.msg.MoveLinResult()
        try:
            success, success_rate, current_pose = self.move_lin(goal.group_name, pose_goal, goal.speed)
        except rospy.ROSException:
            self.move_lin_action.set_aborted()
        goal_res.success = success
        goal_res.success_rate = success_rate*100
        goal_res.current_pose = current_pose
        self.move_lin_action.set_succeeded(goal_res)

    def move_lin(self, group_name, pose_goal_stamped, speed = 1.0, acceleration = 0.0, end_effector_link = ""):
        robots = moveit_commander.RobotCommander()
        planning_scene = moveit_commander.PlanningSceneInterface()
        
        self.publishMarker(pose_goal_stamped, "pose")

        if end_effector_link == "":
            if group_name == 'a_bot':
                end_effector_link = 'a_bot_robotiq_85_tip_link'
            elif group_name == "b_bot":
                end_effector_link = "b_bot_single_suction_gripper_pad_link"
            elif group_name == 'c_bot':
                end_effector_link = 'c_bot_robotiq_85_tip_link'
            elif group_name == "d_bot":
                end_effector_link = "d_bot_dual_suction_gripper_pad_link"

        group = moveit_commander.MoveGroupCommander(group_name)

        group.set_end_effector_link(end_effector_link)
        group.set_pose_target(pose_goal_stamped)
        rospy.logdebug("Setting velocity scaling to " + str(speed))
        group.set_max_velocity_scaling_factor(speed)

        # FIXME: At the start of the program, get_current_pose() did not return the correct value. Should be a bug report.
        waypoints = []
        # waypoints.append(group.get_current_pose().pose)
        pose_goal_world = self.listener.transformPose("world", pose_goal_stamped).pose
        waypoints.append(pose_goal_world)
        (plan, fraction) = group.compute_cartesian_path(waypoints,  # waypoints to follow
                                                        0.0005,       # eef_step
                                                        0.0)        # jump_threshold
        rospy.loginfo("Compute cartesian path succeeded with " + str(fraction*100) + "%")
        plan = group.retime_trajectory(robots.get_current_state(), plan, speed)

        plan_success = group.execute(plan, wait=True)
        group.stop()
        group.clear_pose_targets()
        current_pose = group.get_current_pose()
        current_pose_world = self.listener.transformPose('o2as_ground', current_pose)
        return plan_success, fraction, current_pose_world


if __name__ == '__main__':
    rospy.init_node("aist_skills", anonymous=True)

    try:
        node = SkillServer()
        while not rospy.is_shutdown():
            rospy.sleep(.1)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
