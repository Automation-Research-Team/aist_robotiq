#!/usr/bin/env python

import rospy
from geometry_msgs import msg as gmsg
from aist_routines.base import AISTBaseRoutines

######################################################################
#  class CheckCalibrationRoutines                                    #
######################################################################
class CheckCalibrationRoutines(AISTBaseRoutines):
    """Wrapper of MoveGroupCommander specific for this script"""

    def __init__(self):
        super(CheckCalibrationRoutines, self).__init__()
        self._camera_name = rospy.get_param('~camera_name', 'a_phoxi_m_camera')
        self._robot_name  = rospy.get_param('~robot_name', 'b_bot')

    def move(self, speed):
        self.trigger_frame(self._camera_name)
        marker_pose = rospy.wait_for_message(self._camera_name
                                             + '/aruco_detector/pose',
                                             gmsg.PoseStamped, 10)
        approach_pose = self.effector_target_pose(marker_pose, (0, 0, 0.05))

        #  We have to transform the target pose to reference frame before moving
        #  to the approach pose because the marker pose is given w.r.t. camera
        #  frame which will change while moving in the case of "eye on hand".
        target_pose = self.transform_pose_to_reference_frame(
                          self.effector_target_pose(marker_pose, (0, 0, 0)))
        print('  move to ' + self.format_pose(approach_pose))
        (success, _, current_pose) = self.go_to_pose_goal(self._robot_name,
                                                          approach_pose,
                                                          speed, move_lin=True)
        print('  reached ' + self.format_pose(current_pose))
        rospy.sleep(1)
        print('  move to ' + self.format_pose(target_pose))
        (success, _, current_pose) = self.go_to_pose_goal(self._robot_name,
                                                          target_pose,
                                                          speed, move_lin=True)
        print('  reached ' + self.format_pose(current_pose))

    def run(self, speed):
        self.continuous_shot(self._camera_name, False)
        self.go_to_named_pose('home', self._robot_name)

        while not rospy.is_shutdown():
            try:
                key = raw_input('>> ')
                if key == 'q':
                    break
                elif key == 'h':
                    self.go_to_named_pose('home', self._robot_name)
                else:
                    self.move(speed)
            except rospy.ROSException as ex:
                print ex.message
            except rospy.ROSInterruptException:
                return
            except KeyboardInterrupt:
                return

        self.go_to_named_pose('home', self._robot_name)

######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node('check_calibration')

    with CheckCalibrationRoutines() as routines:
        speed = 0.05
        routines.run(speed)
