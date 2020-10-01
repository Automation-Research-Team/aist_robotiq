#!/usr/bin/env python

import rospy
import moveit_commander
import tf
import tf_conversions
import geometry_msgs.msg

from aist_routines.base import AISTBaseRoutines

def is_num(s):
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True

######################################################################
#  class BinCalibrationRoutines                                      #
######################################################################
class BinCalibrationRoutines(AISTBaseRoutines):
    def __init__(self):
        super(BinCalibrationRoutines, self).__init__()
        self._bins = {
            'bin_1_part_5',
            'bin_1_part_16',
            'bin_1_part_17',
            'bin_2_part_4',
            'bin_2_part_7',
            'bin_2_part_8',
        }
        self._robot_name = 'b_bot'    # set property
        self._axis       = 'Z'
        self._offset     = (0, 0, 0)
        self._speed      = 0.3
        rospy.loginfo('Calibration class is staring up!')

    @property
    def robot_name(self):
        return self._robot_name

    @robot_name.setter
    def robot_name(self, name):
        self._robot_name = name

    @property
    def axis(self):
        return self._axis

    @axis.setter
    def axis(self, a):
        self._axis = a

    @property
    def prompt(self):
        return '{}:({:.4f}, {:.4f}, {:.4f})-{} >> '.format(
            self._robot_name,
            self._offset[0], self._offset[1], self._offset[2], self._axis)

    def go_home(self):
        self.go_to_named_pose(self._robot_name, 'home')

    def go_to(self, target_frame, offset, interactive=True):
        if interactive:
            key = raw_input('  Press `Enter` to move >> ')
            if key == 'q':
                raise Exception('Aborted!')
        self._offset = offset
        (success, _, current_pose) \
            = self.go_to_frame(self._robot_name, target_frame, self._offset,
                               self._speed, move_lin=False)
        p = self.listener.transformPose('workspace_center',
                                        current_pose).pose.position
        self._offset = (p.x, p.y, p.z)
        return success

    def touch_the_table(self):
        print('**** Calibrating between robot and table. ****')
        self.go_to_named_pose(self._robot_name, 'home')
        print('==== Going to 3 cm above the table. ====')
        self.go_to('workspace_center', (0, 0, 0.03))
        print('==== Going to 1 cm above the table. ====')
        self.go_to('workspace_center', (0, 0, 0.01))
        self.go_home()

    def bin_calibration(self):
        print('**** Calibrating bins. ****')
        self.go_to_named_pose(self._robot_name, 'home')
        for bin in self._bins:
            print('==== Going to 3 cm above center of bin. ====')
            self.go_to(bin, (0, 0, 0.03))
        self.go_home()

    def bin_corner_calibration(self):
        print('**** Calibrating bin corners. ****')
        self.go_to_named_pose(self._robot_name, 'home')
        for bin in self._bins:
            print('---- ' + bin + ' ----')
            target_frame = bin + '_top_back_left_corner'
            self.go_to(target_frame, (0, 0, 0.02))
            target_frame = bin + '_top_back_right_corner'
            self.go_to(target_frame, (0, 0, 0.02))
            target_frame = bin + '_top_front_right_corner'
            self.go_to(target_frame, (0, 0, 0.02))
            target_frame = bin + '_top_front_left_corner'
            self.go_to(target_frame, (0, 0, 0.02))
        self.go_home()

    def workspace_calibration(self):
        print('**** Calibrating workspace. ****')
        self.go_to_named_pose(self._robot_name, 'home')
        self.go_to('workspace_center', ( 0,    0, 0.03))
        self.go_to('workspace_center', (-0.10, 0, 0.03))
        self.go_to('workspace_center', (-0.20, 0, 0.03))
        self.go_to('workspace_center', (-0.25, 0, 0.03))
        self.go_to('workspace_center', ( 0,    0, 0.03))
        self.go_to('workspace_center', ( 0.10, 0, 0.03))
        self.go_to('workspace_center', ( 0.20, 0, 0.03))
        self.go_to('workspace_center', ( 0.25, 0, 0.03))
        self.go_home()

    def move_along_axis(self, val):
        if self._axis == 'X':
            offset = (val, self._offset[1], self._offset[2])
        elif self._axis == 'Y':
            offset = (self._offset[0], val, self._offset[2])
        elif self._axis == 'Z':
            offset = (self._offset[0], self._offset[1], val)
        self.go_to('workspace_center', offset, False)

if __name__ == '__main__':

    rospy.init_node('bin_calibration', anonymous=True)

    with BinCalibrationRoutines() as calib:
        while not rospy.is_shutdown():
            print('============ Calibration procedures ============ ')
            print('  [A|B|C|D]: Switch to a_bot, b_bot, c_bot or d_bot')
            print('  [X|Y|Z]: Switch to x-axis, y-axis or z-axis')
            print('  h: Go home')
            print('  t: Touch the table (workspace_center)')
            # print('  b: Bin center calibration')
            print('  c: Bin corner calibration')
            print('  w: Workspace calibration')
            print('  q: Quit')

            key = raw_input(calib.prompt)
            if key == 'A':
                calib.robot_name = 'a_bot'
            elif key == 'B':
                calib.robot_name = 'b_bot'
            elif key == 'C':
                calib.robot_name = 'c_bot'
            elif key == 'D':
                calib.robot_name = 'd_bot'
            elif key == 'X':
                calib.axis = 'X'
            elif key == 'Y':
                calib.axis = 'Y'
            elif key == 'Z':
                calib.axis = 'Z'
            elif key == 'h':
                calib.go_to_named_pose(calib.robot_name, 'home')
            elif key == 't':
                calib.touch_the_table()
            # elif key == 'b':
            #     calib.bin_calibration()
            elif key == 'c':
                calib.bin_corner_calibration()
            elif key == 'w':
                calib.workspace_calibration()
            elif is_num(key):
                calib.move_along_axis(float(key))
            elif key == 'q':
                calib.go_home()
                break
