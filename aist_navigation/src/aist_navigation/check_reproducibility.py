#!/usr/bin/env python
from math import radians, degrees
import math
import sys
import os
import datetime
import copy

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import geometry_msgs.msg as gmsg
import tf.transformations as tfs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from skimage import io
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import threading



class FollowTrajectoryClient(object):
    """Based on pick_and_place_demo on fetchrobotics/fetch_gazebo_demo
    """

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        rospy.loginfo("%s is connected!" % name)
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()


class HeadControllerClient(object):
    def __init__(self):
        self.client = FollowTrajectoryClient(
            "head_controller", ["head_pan_joint", "head_tilt_joint"])

    def move_to(self, pan_angle, tilt_angle):
        self.client.move_to([pan_angle, tilt_angle])


class MoveBaseClient(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "/move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("move_base action is connected!")
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.current_pose = gmsg.Pose()
        self.odom_recv_event = threading.Event()

        self.odom_err_pub = rospy.Publisher(
            "/odom_error", gmsg.Pose2D, queue_size=1)
        self.odom_correct_pub = rospy.Publisher(
            "/odom_correct", gmsg.Pose, queue_size=1)

        #
        # for error correction
        #
        self.cumlative_error_u = 0.0
        self.cumlative_error_v = 0.0
        self.cumlative_error_l = 0.0
        self.cumlative_error_t_delta = 0.0
        self.cumlative_error_theta = 0.0

    def odom_callback(self, odom):
        self.odom_recv_event.clear()
        self.current_pose = copy.deepcopy(odom.pose.pose)
        self.odom_recv_event.set()

    #
    # wait for updating self.current_pose
    #
    # return: self.current_pose (geometry_msgs/Pose)
    #
    def wait_for_pose_update(self, timeout=None):
        self.odom_recv_event.clear()
        self.odom_recv_event.wait(timeout)
        self.odom_correct_pub.publish(self.current_pose)
        return self.current_pose

    def move_to(self, x, y, theta, frame="map"):
        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = frame
        goal_pose.target_pose.pose.position = gmsg.Vector3(x, y, 0)
        goal_pose.target_pose.pose.orientation = gmsg.Quaternion(
            *tfs.quaternion_from_euler(0, 0, theta))

        return self.client.send_goal_and_wait(goal_pose)

    #
    # get error between given odometoric pose with destinations pose
    #
    # odom_pose: odometoric pose (geometry_msgs/Pose) or None (current pose)
    # normalize: normalize each element by destination distance or rotaion
    #            angle from source position
    #
    # return: error vector alonging with direction of travel
    #         (geometry_msgs/Pose2d)
    #         .x: delta along with direction of travel
    #         .y: delta orthogonalized with direction of travel
    #         .theta: rotation
    #
    #
    def get_odom_error(self, odom_pose=None, normalize=True):
        # MIN_CUMLATIVE_ERROR_L=sys.float_info.epsilon
        # MIN_CUMLATIVE_ERROR_THETA=sys.float_info.epsilon
        MIN_CUMLATIVE_ERROR_L = 0.25  # 10cm
        MIN_CUMLATIVE_ERROR_THETA = math.pi / 4  # 45deg

        #
        # IF source or destinations positions do not exist,
        # return zero vector.
        #
        try:
            tmp = self.src_pose2d
            tmp = self.dst_pose2d
        except AttributeError:
            return gmsg.Pose2D(0.0, 0.0, 0.0)

        if odom_pose is None:
            odom_pose = copy.deepcopy(self.wait_for_pose_update())

        #
        # src to dst vec
        #
        vx = self.dst_pose2d.x - self.src_pose2d.x
        vy = self.dst_pose2d.y - self.src_pose2d.y
        vl = math.sqrt(vx * vx + vy * vy)
        if(vl >= sys.float_info.epsilon):
            vx /= vl
            vy /= vl
        else:
            vx = vy = vl = 0.0

        #
        # error vec (u, v) alonging with direction of travel
        #
        # Solve for (u, v)
        #   (x1 + a u) - b v = x2
        #   (y1 + b u) + a v = y2
        #
        # where
        #   (x1, y1): destination point
        #   (x2, y2): reaching point (odometer value)
        #   (a, b)  : src to dst vector
        #
        #
        ex = dx = odom_pose.position.x - self.dst_pose2d.x
        ey = dy = odom_pose.position.y - self.dst_pose2d.y

        # try:
        #    (ex,ey)=np.linalg.solve(np.array([[vx,-vy],[vy,vx]]),
        #                            np.array([dx,dy]))
        # except np.linalg.linalg.LinAlgError:
        #    # nop

        #
        # always vx^2 +vy^2 = 1
        #
        # np.invert(np.array([[vx,-vy],[vy,vx]])) #=>
        #    np.array([[vx,vy],[-vy,vx]])
        #
        if(vl >= sys.float_info.epsilon):
            ex = vx * dx + vy * dy
            ey = -vy * dx + vx * dy

        self.cumlative_error_u += ex
        self.cumlative_error_v += ey
        self.cumlative_error_l += vl

        #
        # rotation error
        #
        c_theta = tfs.euler_from_quaternion((odom_pose.orientation.x,
                                           odom_pose.orientation.y,
                                           odom_pose.orientation.z,
                                           odom_pose.orientation.w))[2]
        #
        # src to dst rotation
        #
        # vt = math.acos(self.dst_pose2d.theta - self.src_pose2d.theta)
        vt = self.dst_pose2d.theta - self.src_pose2d.theta
        vt = math.atan2(math.sin(vt), math.cos(vt))  # normalization
        if(vt < 0.0):
            vt = -vt

        #
        # dst to current rotation
        #
        et = c_theta - self.dst_pose2d.theta
        et=math.atan2(math.sin(et), math.cos(et))  # normalization

        self.cumlative_error_t_delta += et
        # self.cumlative_error_theta+=vt
        self.cumlative_error_theta += 1.0

        if(normalize):
            if(self.cumlative_error_l >= MIN_CUMLATIVE_ERROR_L):
                ex=self.cumlative_error_u / self.cumlative_error_l
                ey=self.cumlative_error_v / self.cumlative_error_l
            else:
                ex=ey=0.0

            # if(self.cumlative_error_theta>=MIN_CUMLATIVE_ERROR_THETA):
            #    et=self.cumlative_error_t_delta/self.cumlative_error_theta
            # else:
            #    et=0.0
            et=self.cumlative_error_t_delta / self.cumlative_error_theta
        self.odom_err_pub.publish(gmsg.Pose2D(ex, ey, et))
        return gmsg.Pose2D(ex, ey, et)


    def move_to_with_error_correction(self, x, y, theta, frame = "map"):
        pose=copy.deepcopy(self.wait_for_pose_update())
        e=self.get_odom_error(pose)  # get normalized error

        #
        # recode start position
        #
        self.src_pose2d=gmsg.Pose2D(pose.position.x,
                                    pose.position.y,
                                    tfs.euler_from_quaternion(
                                        (pose.orientation.x,
                                         pose.orientation.y,
                                         pose.orientation.z,
                                         pose.orientation.w))[2])

        #######################################################################
        #
        # position error correction
        #

        # src to goal vector
        dx=x - self.src_pose2d.x
        dy=y - self.src_pose2d.y

        dst_x=x
        dst_y=y
        if(abs(dx) >= sys.float_info.epsilon or
           abs(dy)>=sys.float_info.epsilon):
            #
            # solve length between source to destination L0
            #
            # tangent length L1 between src (x0, y0)
            # and goal circle (x - x1)^2 + (y - y1)^2 = r^2
            #
            # L1^2 = (x0 - x1)^2 + (y0 - y1)^2 - r^2
            # r^2 = (e.y * L0)^2
            #
            # L1 =  L0 + L0 * e.x
            #
            l0=math.sqrt((dx*dx+dy*dy)/(1.0+2*e.x+e.x*e.x+e.y*e.y))
            l1=l0+l0*e.x

            #
            # Solve for (a, b); src to dst vec
            #   x1 = x0 + L1 * a - e.y * L0 * b
            #   y1 = y0 + L1 * b + e.y * L0 * a
            #
            a=b=0.0
            try:
                (a,b)=np.linalg.solve(np.array([[l1,-e.y*l0],
                                                [e.y*l0,l1]]),
                                      np.array([dx,dy]))
            except np.linalg.linalg.LinAlgError:
                a=b=0.0

            dst_x=self.src_pose2d.x+a*l0
            dst_y=self.src_pose2d.y+b*l0

        ########################################################################
        #
        # rotation error collection
        #
        s_theta_x=math.cos(self.src_pose2d.theta)
        s_theta_y=math.sin(self.src_pose2d.theta)
        d_theta_x=math.cos(theta)
        d_theta_y=math.sin(theta)

        d=math.acos(s_theta_x*d_theta_x+s_theta_y*d_theta_y)+e.theta
        dst_theta=self.src_pose2d.theta+d
        dst_theta=math.atan2(math.sin(dst_theta),math.cos(dst_theta))

        #
        # recode destinations position
        #
        self.dst_pose2d=gmsg.Pose2D(dst_x,dst_y,dst_theta)

        return self.move_to(dst_x, dst_y, dst_theta, frame)

def format_pose(pose):
    rpy = map(degrees, tfs.euler_from_quaternion(
        (pose.orientation.x,
         pose.orientation.y,
         pose.orientation.z,
         pose.orientation.w)))
    return gmsg.Pose2D(pose.position.x, pose.position.y, rpy[2])


if __name__ == "__main__":
    rospy.init_node('ReproducibilityCheck', disable_signals=True)
    dirpath = os.path.join(
        os.getenv('HOME'), '.ros/check_base_reproducibility', datetime.datetime.now().strftime("%Y%m%d%H%M%S"))
    if not os.path.exists(dirpath):
        os.makedirs(dirpath)
    idx = 0

    target_poses = {
        "shelf": [0, 0, 0],
        "table": [0.2, -2.2, radians(180)]
    }

    wp_theta_outward=math.atan2(-2.2,0.2)
    way_points_outward = [
        [0, 0, wp_theta_outward],
        [0.1, -1.1, wp_theta_outward],
        [0.2, -2.2, wp_theta_outward]
    ]
    wp_theta_homeward=math.atan2(2.2,-0.2)
    way_points_homeward = [
        [0.2, -2.2, wp_theta_homeward],
        [0.1, -1.1, wp_theta_homeward],
        [0, 0, wp_theta_homeward]
    ]

    odom_filename = dirpath + '/odom.csv'
    odom_file = open(odom_filename, mode='w')
    odom_file.write('position_idx,time,req_x,req_y,req_theta,actual_x,actual_y,actual_theta,e-collect_x,e-collect_y,e-collect_theta,error_x,error_y,error_theta\n')
    try:
        base_client = MoveBaseClient()
        head_client = HeadControllerClient()

        #
        # move to initial position
        #
        _, e = base_client.move_to_with_error_correction(*target_poses["shelf"],
                                                  frame="odom")

        # image_saver = RGBDSaver()

        pose=base_client.wait_for_pose_update()
        actual_pose = format_pose(pose)
        odom_file.write("shelf_{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
            idx, rospy.Time.now(),
            target_poses["shelf"][0], target_poses["shelf"][1], target_poses["shelf"][2],
            actual_pose.x, actual_pose.y, actual_pose.theta,
            base_client.dst_pose2d.x, base_client.dst_pose2d.y, base_client.dst_pose2d.theta,
            e.x, e.y, e.theta
        ))
        print actual_pose
        iidx = 1
        while not rospy.is_shutdown():
            for wp in way_points_outward:
                _, e = base_client.move_to_with_error_correction(*wp, frame="odom")
                pose=base_client.wait_for_pose_update()
                actual_pose = format_pose(pose)
                odom_file.write("outward_{}-{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                    idx, iidx, rospy.Time.now(),
                    wp[0], wp[1], wp[2],
                    actual_pose.x, actual_pose.y, actual_pose.theta,
                    base_client.dst_pose2d.x, base_client.dst_pose2d.y, base_client.dst_pose2d.theta,
                    e.x, e.y, e.theta
                ))
                print actual_pose
                iidx+=1


            _, e = base_client.move_to_with_error_correction(*target_poses["table"],
                                                      frame="odom")
            head_client.move_to(0, radians(30))

            # image_saver.save(dirpath + "/table_" + str(idx))
            pose=base_client.wait_for_pose_update()
            actual_pose = format_pose(pose)
            odom_file.write("table_{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                idx, rospy.Time.now(),
                target_poses["table"][0], target_poses["table"][1], target_poses["table"][2],
                actual_pose.x, actual_pose.y, actual_pose.theta,
                base_client.dst_pose2d.x, base_client.dst_pose2d.y, base_client.dst_pose2d.theta,
                e.x, e.y, e.theta
            ))
            print actual_pose

            iidx = 0
            for wp in way_points_homeward:
                _, e = base_client.move_to_with_error_correction(*wp, frame="odom")
                pose=base_client.wait_for_pose_update()
                actual_pose = format_pose(pose)
                odom_file.write("homeward_{}-{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                    idx, iidx, rospy.Time.now(),
                    wp[0], wp[1], wp[2],
                    actual_pose.x, actual_pose.y, actual_pose.theta,
                    base_client.dst_pose2d.x, base_client.dst_pose2d.y, base_client.dst_pose2d.theta,
                    e.x, e.y, e.theta
                ))
                print actual_pose
                iidx += 1


            _, e = base_client.move_to_with_error_correction(*target_poses["shelf"],
                                                      frame="odom")
            head_client.move_to(0, radians(30))
            # image_saver.save(dirpath + "/shelf_" + str(idx))
            pose=base_client.wait_for_pose_update()
            actual_pose = format_pose(pose)
            odom_file.write("shelf_{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n".format(
                idx, rospy.Time.now(),
                target_poses["shelf"][0], target_poses["shelf"][1], target_poses["shelf"][2],
                actual_pose.x, actual_pose.y, actual_pose.theta,
                base_client.dst_pose2d.x, base_client.dst_pose2d.y, base_client.dst_pose2d.theta,
                e.x, e.y, e.theta
            ))
            print actual_pose
            idx += 1

    except (rospy.ROSInitException, rospy.ROSInternalException) as e:
        rospy.logerr("ReproducibilityCheck has occured: ".format(e.message))
    except rospy.ROSInterruptException:
        pass
    finally:
        odom_file.close()

    rospy.signal_shutdown("ReproducibilityCheck completed.")
