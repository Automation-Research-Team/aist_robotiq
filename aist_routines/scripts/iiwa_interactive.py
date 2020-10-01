#!/usr/bin/env python

import sys
import os
import copy
import rospy
import argparse
from geometry_msgs import msg as gmsg

from moveit_commander import PlanningSceneInterface

from tf import transformations as tfs
from math import radians, degrees
from aist_routines.iiwa import IiwaRoutines

######################################################################
#  global functions                                                  #
######################################################################
def is_num(s):
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True

######################################################################
#  class InteractiveRoutines                                         #
######################################################################
class InteractiveRoutines(IiwaRoutines):
    refposes = {
        'a_iiwa': [0.00,  0.00, 2.10, radians(0), radians(0), radians(0)],
        'b_iiwa': [0.00,  1.20, 2.10, radians(0), radians(0), radians(0)],
    }
    refposes2 = {
        # 'a_iiwa': [0.70, -0.20, 1.50, radians(0), radians(0), radians(0)],
        'a_iiwa': [0.30,  0.00, 2.00, radians(0), radians(0), radians(0)],
        'b_iiwa': [0.70,  1.40, 1.50, radians(0), radians(0), radians(0)],
    }

    def __init__(self, robot_name, camera_name, speed, ns):
        super(InteractiveRoutines, self).__init__(ns)

        self._robot_name   = robot_name
        self._camera_name  = camera_name
        self._speed        = speed

    def go_standing(self):
        self.go_to_named_pose(self._robot_name, 'standing')

    def go_home(self):
        self.go_to_named_pose(self._robot_name, 'home')

    """
    def go_back(self):
        self.go_to_named_pose(self._robot_name, 'back')
    """

    def move(self, pose):
        target_pose = gmsg.PoseStamped()
        # target_pose.header.frame_id = "workspace_center"
        target_pose.header.frame_id = "world"

        if type(pose) is list:
            target_pose.pose = gmsg.Pose(
                gmsg.Point(pose[0], pose[1], pose[2]),
                gmsg.Quaternion(
                    *tfs.quaternion_from_euler(pose[3], pose[4], pose[5])))
        elif type(pose) is gmsg.Pose:
            target_pose.pose = pose

        (success, _, current_pose) = self.go_to_pose_goal(
                                                self._robot_name, target_pose,
                                                self._speed,
                                                move_lin=False)
        return success

    def create_objects(self, scene, robot_commander, z_ext):
        scene.remove_world_object()

        p = gmsg.PoseStamped()
        p.header.frame_id = robot_commander.get_planning_frame()

        p.pose.position.x =  0.8
        p.pose.position.y = -0.2
        p.pose.position.z =  0.5 + z_ext
        scene.add_box("part1", p, (0.03, 0.03, 0.3))

        p.pose.position.x =  0.8
        p.pose.position.y =  1.4
        p.pose.position.z =  0.5 + z_ext
        scene.add_box("part2", p, (0.03, 0.03, 0.3))

    def pose_to_object(self, scene, robot_commander, robot_name):
        obj_name = ''
        link_0 = None
        if robot_name == 'a_iiwa':
            obj_name = 'part1'
            link_0 = robot_commander.get_link('a_iiwa_link_0')
        elif robot_name == 'b_iiwa':
            obj_name = 'part2'
            link_0 = robot_commander.get_link('b_iiwa_link_0')
        if len(obj_name) <= 0:
            print '# pose_to_object # no target'
            return
        if link_0 is None:
            print '# pose_to_object # no link_0'
            return
        print '# pose_to_object # target object :', obj_name

        link_0_pose = link_0.pose()
        q_tf = tfs.quaternion_from_euler(0, 3.14/2, 0)

        objs = scene.get_objects()
        if obj_name in objs:
            poses = objs[obj_name].primitive_poses
            print '# pose_to_object #\n', poses
            for pose in poses:
                if pose.position.x > link_0_pose.pose.position.x:
                    pose.position.x -= 0.1
                else:
                    pose.position.x += 0.1
                pose.position.z += 0.2
                pose.orientation = gmsg.Quaternion(q_tf[0], q_tf[1], q_tf[2], q_tf[3])

                self.move(pose)

    def run(self):
        scene = PlanningSceneInterface()
        rospy.sleep(1)

        # Reset pose
        # self.go_home()
        self.go_standing()

        axis = 'Y'

        _pose = None

        while not rospy.is_shutdown():
            current_pose = self.get_current_pose(self._robot_name)
            print "# current_pose #\n", current_pose
            prompt = "{:>5}:{}{:>9}>> " \
                   .format(axis, self.format_pose(current_pose), "moveit")

            key = raw_input(prompt)

            if key == 'q':
                break
            elif key == 'X':
                axis = 'X'
            elif key == 'Y':
                axis = 'Y'
            elif key == 'Z':
                axis = 'Z'
            elif key == 'R':
                axis = 'Roll'
            elif key == 'P':
                axis = 'Pitch'
            elif key == 'W':
                axis = 'Yaw'
            elif key == '+':
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] += 0.01
                elif axis == 'Y':
                    goal_pose[1] += 0.01
                elif axis == 'Z':
                    goal_pose[2] += 0.01
                elif axis == 'Roll':
                    goal_pose[3] += radians(10)
                elif axis == 'Pitch':
                    goal_pose[4] += radians(10)
                else:
                    goal_pose[5] += radians(10)
                self.move(goal_pose)
            elif key == '-':
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] -= 0.01
                elif axis == 'Y':
                    goal_pose[1] -= 0.01
                elif axis == 'Z':
                    goal_pose[2] -= 0.01
                elif axis == 'Roll':
                    goal_pose[3] -= radians(10)
                elif axis == 'Pitch':
                    goal_pose[4] -= radians(10)
                else:
                    goal_pose[5] -= radians(10)
                self.move(goal_pose)
            elif is_num(key):
                if _pose is None:
                    _pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    _pose[0] = float(key)
                elif axis == 'Y':
                    _pose[1] = float(key)
                elif axis == 'Z':
                    _pose[2] = float(key)
                elif axis == 'Roll':
                    _pose[3] = radians(float(key))
                elif axis == 'Pitch':
                    _pose[4] = radians(float(key))
                elif axis == 'Yaw':
                    _pose[5] = radians(float(key))
                print "# _pose #\n", _pose
            elif key == 'reset':
                _pose = None
            elif key == 'go':
                self.move(_pose)
            elif key == 's':
                self.stop(self._robot_name)
            elif key == 'pregrasp':
                self.pregrasp(self._robot_name)
            elif key == 'grasp':
                self.grasp(self._robot_name)
            elif key == 'release':
                self.release(self._robot_name)
            elif key == 'cont':
                self.continuous_shot(self._camera_name, True)
            elif key == 'stopcont':
                self.continuous_shot(self._camera_name, False)
            elif key == 'trigger':
                self.trigger_frame(self._camera_name)
            elif key == 'mask':
                self.create_mask_image(self._camera_name,
                                       int(raw_input("  #bins? ")))
            elif key == 'search':
                self.delete_all_markers()
                self.graspability_send_goal(self._robot_name,
                                            self._camera_name, 4, 0, True)
                (poses, gscore, success) = \
                    self.graspability_wait_for_result(self._camera_name, 0)
                for gs in gscore:
                    print(str(gs))
                print(str(poses))
            elif key == 'o':
                self.move(InteractiveRoutines.refposes[self._robot_name])
            elif key == 'pose':
                self.move(InteractiveRoutines.refposes2[self._robot_name])
            elif key == 'standing':
                self.go_standing()
            elif key == 'h':
                self.go_home()
            elif key == 'show_objs':
                self.create_objects(scene, self._cmd, 0.80)
            elif key == 'remove_objs':
                scene.remove_world_object()
            elif key == 'pose_obj':
                self.pose_to_object(scene, self._cmd, self._robot_name)
            """
            elif key == 'b':
                self.go_back()
            """

        # Reset pose
        # self.go_home()
        self.go_standing()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Perform tool calibration')
    parser.add_argument('-r',
                        '--robot_name',
                        action='store',
                        nargs='?',
                        default='a_iiwa',
                        type=str,
                        choices=None,
                        help='robot name',
                        metavar=None)
    parser.add_argument('-c',
                        '--camera_name',
                        action='store',
                        nargs='?',
                        default='a_phoxi_m_camera',
                        type=str,
                        choices=None,
                        help='camera name',
                        metavar=None)
    parser.add_argument('-n',
                        '--ns',
                        action='store',
                        nargs='?',
                        default='',
                        type=str,
                        choices=None,
                        help='namespace',
                        metavar=None)
    args = parser.parse_args()

    speed = 0.1
    with InteractiveRoutines(args.robot_name,
                             args.camera_name, speed, args.ns) as routines:
        routines.run()
