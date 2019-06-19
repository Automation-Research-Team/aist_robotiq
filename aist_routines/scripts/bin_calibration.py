#!/usr/bin/env python

import rospy
import moveit_commander
import tf
import tf_conversions
import geometry_msgs.msg

from aist_routines.base import AISTBaseRoutines

######################################################################
#  class BinCalibrationRoutines                                      #
######################################################################
class BinCalibrationRoutines(AISTBaseRoutines):
    def __init__(self, robot_name, speed):
        super(BinCalibrationRoutines, self).__init__()
        self._bins = {
            "bin_1_part_15",
            "bin_1_part_16",
            "bin_1_part_17",
            "bin_2_part_4",
            "bin_2_part_7",
            "bin_2_part_8",
        }
        self._speed     = speed
        self.robot_name = robot_name    # set property
        rospy.loginfo("Calibration class is staring up!")

    @property
    def robot_name(self):
        return self._group.get_name()

    @robot_name.setter
    def robot_name(self, name):
        self._group = moveit_commander.MoveGroupCommander(name)
        self._group.set_pose_reference_frame("workspace_center")
        if self.gripper(name).type == "suction":
            self.gripper(name).release()
        else:
            self.gripper(name).grasp()

    def go_home(self):
        raw_input("  Press `Enter` to go home >> ")
        self.go_to_named_pose("home", self.robot_name)

    def go_to(self, target_frame, offset):
        key = raw_input("  Press `Enter` to move >> ")
        if key == "q":
            raise Exception("Aborted!")
        (success, _, current_pose) \
            = self.go_to_frame(self.robot_name, target_frame, offset,
                               self._speed, move_lin=True)
        return success

    def touch_the_table(self):
        print("**** Calibrating between robot and table. ****")
        self.go_to_named_pose("home", self.robot_name)
        print("==== Going to 3 cm above the table. ====")
        self.go_to("workspace_center", (0, 0, 0.03))
        print("==== Going to 1 cm above the table. ====")
        self.go_to("workspace_center", (0, 0, 0.01))
        self.go_home()

    def bin_calibration(self):
        print("**** Calibrating bins. ****")
        self.go_to_named_pose("home", self.robot_name)
        for bin in self._bins:
            print("==== Going to 3 cm above center of bin. ====")
            self.go_to(bin, (0, 0, 0.03))
        self.go_home()

    def bin_corner_calibration(self):
        print("**** Calibrating bin corners. ****")
        self.go_to_named_pose("home", self.robot_name)
        for bin in self._bins:
            print("---- " + bin + " ----")
            target_frame = bin + "_top_back_left_corner"
            self.go_to(target_frame, (0, 0, 0.005))
            target_frame = bin + "_top_back_right_corner"
            self.go_to(target_frame, (0, 0, 0.005))
            target_frame = bin + "_top_front_right_corner"
            self.go_to(target_frame, (0, 0, 0.005))
            target_frame = bin + "_top_front_left_corner"
            self.go_to(target_frame, (0, 0, 0.005))
        self.go_home()

    def workspace_calibration(self):
        print("**** Calibrating workspace. ****")
        self.go_to_named_pose("home", self.robot_name)
        self.go_to("workspace_center", ( 0,    0, 0.03))
        self.go_to("workspace_center", (-0.10, 0, 0.03))
        self.go_to("workspace_center", (-0.20, 0, 0.03))
        self.go_to("workspace_center", (-0.25, 0, 0.03))
        self.go_to("workspace_center", ( 0,    0, 0.03))
        self.go_to("workspace_center", ( 0.10, 0, 0.03))
        self.go_to("workspace_center", ( 0.20, 0, 0.03))
        self.go_to("workspace_center", ( 0.25, 0, 0.03))
        self.go_home()

    def workspace_calibration_interactive(self):
        print("*** Calibrating workspace interactively. ****")
        x = float(raw_input("x >> "))
        y = float(raw_input("y >> "))
        z = float(raw_input("z >> "))
        self.go_to("workspace_center", (x, y, z))


if __name__ == "__main__":

    with BinCalibrationRoutines("b_bot", 0.3) as calib:
        while not rospy.is_shutdown():
            print("============ Calibration procedures ============ ")
            print("  [A|B|C|D]: Switch to a_bot, b_bot, c_bot or d_bot")
            print("  h: Go home")
            print("  t: Touch the table (workspace_center)")
            # print("  b: Bin center calibration")
            print("  c: Bin corner calibration")
            print("  w: Workspace calibration")
            print("  q: Quit")

            key = raw_input(calib.robot_name + ">> ")
            if key == 'A':
                calib.robot_name = "a_bot"
            elif key == 'B':
                calib.robot_name = "b_bot"
            elif key == 'C':
                calib.robot_name = "c_bot"
            elif key == 'D':
                calib.robot_name = "d_bot"
            elif key == 'h':
                calib.go_to_named_pose("home", calib.robot_name)
            elif key == 't':
                calib.touch_the_table()
            # elif key == 'b':
            #     calib.bin_calibration()
            elif key == 'c':
                calib.bin_corner_calibration()
            elif key == 'w':
                calib.workspace_calibration()
            elif key == 'q':
                break
