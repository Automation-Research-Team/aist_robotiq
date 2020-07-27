#!/usr/bin/env python

import rospy
from geometry_msgs      import msg as gmsg
from tf                 import transformations as tfs
from math               import radians
from aist_routines.ur   import URRoutines
from aist_routines.base import AISTBaseRoutines

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
class InteractiveRoutines(AISTBaseRoutines):
    refposes = {
        'a_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians( 90)],
        'b_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians(-90)],
        'c_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians( 90)],
        'd_bot': [0.00, 0.00, 0.3, radians(  0), radians( 90), radians(  0)],
    }

    def __init__(self):
        super(InteractiveRoutines, self).__init__()

        self._robot_name  = rospy.get_param('~robot_name', 'b_bot')
        self._camera_name = rospy.get_param('~camer_name', 'a_phoxi_m_camera')
        self._speed       = rospy.get_param('~speed',       0.1)
        self._ur_movel    = False

    def move(self, pose):
        target_pose = gmsg.PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.pose = gmsg.Pose(
            gmsg.Point(pose[0], pose[1], pose[2]),
            gmsg.Quaternion(
                *tfs.quaternion_from_euler(pose[3], pose[4], pose[5])))
        if self._ur_movel:
            (success, _, current_pose) = self.ur_movel(self._robot_name,
                                                       target_pose,
                                                       velocity=self._speed)
        else:
            (success, _, current_pose) = self.go_to_pose_goal(
                                                self._robot_name, target_pose,
                                                self._speed,
                                                move_lin=True)
        return success

    def run(self):
        # Reset pose
        self.go_home()

        axis = 'Y'

        while not rospy.is_shutdown():
            current_pose = self.get_current_pose(self._robot_name)
            prompt = '{:>5}:{}{:>9}>> ' \
                   .format(axis, self.format_pose(current_pose),
                           'urscript' if self._ur_movel else 'moveit')

            key = raw_input(prompt)

            if key == 'q':
                break
            elif key == 'r':
                self._robot_name = raw_input('  robot name? ')
            elif key == 'c':
                self._camera_name = raw_input('  camera name? ')
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
                goal_pose = self.xyz_rpy(current_pose)
                if axis == 'X':
                    goal_pose[0] = float(key)
                elif axis == 'Y':
                    goal_pose[1] = float(key)
                elif axis == 'Z':
                    goal_pose[2] = float(key)
                elif axis == 'Roll':
                    goal_pose[3] = radians(float(key))
                elif axis == 'Pitch':
                    goal_pose[4] = radians(float(key))
                else:
                    goal_pose[5] = radians(float(key))
                self.move(goal_pose)
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
                                       int(raw_input('  #bins? ')))
            elif key == 'search':
                self.delete_all_markers()
                self.graspability_send_goal(self._robot_name,
                                            self._camera_name, 4, 0, True)
                (poses, gscore, success) = \
                    self.graspability_wait_for_result(self._camera_name, 0)
                for gs in gscore:
                    print(str(gs))
                print(str(poses))
            elif key == 'ur':
                self._ur_movel = not self._ur_movel
            elif key == 'push':
                self.ur_linear_push(self._robot_name, wait=False)
            elif key == 'spiral':
                self.ur_spiral_motion(self._robot_name, wait=False)
            elif key == 'insertion':
                self.ur_insertion(self._robot_name, wait=False)
            elif key == 'hinsertion':
                self.ur_horizontal_insertion(self._robot_name, wait=False)
            elif key == 'spiral':
                self.ur_spiral_motion(self._robot_name, wait=False)
            elif key == 'o':
                self.move(InteractiveRoutines.refposes[self._robot_name])
            elif key == 'h':
                self.go_to_named_pose("home", self._robot_name)
            elif key == 'b':
                self.go_to_named_pose("back", self._robot_name)
            elif key == 'n':
                pose_name = raw_input("  pose name? ")
                try:
                    self.go_to_named_pose(pose_name, self._robot_name)
                except rospy.ROSException as e:
                    rospy.logerr('Unknown pose: %s' % e)
            elif key == 'f':
                frame = raw_input("  frame? ")
                self.go_to_frame(self._robot_name, frame)

        # Reset pose
        self.go_home()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':

    rospy.init_node('interactive', anonymous=True)

    with InteractiveRoutines() as routines:
        routines.run()
