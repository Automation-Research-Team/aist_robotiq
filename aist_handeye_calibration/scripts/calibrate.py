#!/usr/bin/env python

import sys, os, rospy, copy
from math import radians, degrees
from std_srvs.srv  import Empty, Trigger
from geometry_msgs import msg as gmsg
from tf import TransformListener, transformations as tfs
from aist_handeye_calibration.srv import GetSampleList, ComputeCalibration
from aist_routines.base import AISTBaseRoutines

######################################################################
#  class HandEyeCalibrationRoutines                                  #
######################################################################
class HandEyeCalibrationRoutines(AISTBaseRoutines):
    def __init__(self):
        super(HandEyeCalibrationRoutines, self).__init__()

        self._camera_name          = rospy.get_param("~camera_name",
                                                     "a_phoxi_m_camera")
        self._robot_name           = rospy.get_param("~robot_name", "b_bot")
        self._eye_on_hand          = rospy.get_param("~eye_on_hand", False)
        self._robot_base_frame     = rospy.get_param("~robot_base_frame",
                                                     "workspace_center")
        self._robot_effector_frame = rospy.get_param("~robot_effector_frame",
                                                     "b_bot_ee_link")
        self._initpose             = rospy.get_param("~initpose", [])
        self._keyposes             = rospy.get_param("~keyposes", [])
        self._speed                = rospy.get_param("~speed", 1)
        self._sleep_time           = rospy.get_param("~sleep_time", 2)

        if rospy.get_param("calibration", True):
            ns = "/handeye_calibrator/"
            self.get_sample_list = rospy.ServiceProxy(ns + "get_sample_list",
                                                      GetSampleList)
            self.take_sample = rospy.ServiceProxy(ns + "take_sample", Trigger)
            self.compute_calibration = rospy.ServiceProxy(
                ns + "compute_calibration", ComputeCalibration)
            self.save_calibration = rospy.ServiceProxy(ns + "save_calibration",
                                                       Trigger)
            self.reset = rospy.ServiceProxy(ns + "reset", Empty)
        else:
            self.get_sample_list     = None
            self.take_sample         = None
            self.compute_calibration = None
            self.save_calibration    = None
            self.reset               = None

    def is_eye_on_hand(self):
        return self._camera_name.find(self._robot_name) == 0

    def go_home(self):
        self.go_to_named_pose("home", self._robot_name)

    def move(self, pose):
        poseStamped = gmsg.PoseStamped()
        poseStamped.header.frame_id = self._robot_base_frame
        poseStamped.pose = gmsg.Pose(gmsg.Point(*pose[0:3]),
                                     gmsg.Quaternion(
                                         *tfs.quaternion_from_euler(
                                             *map(radians, pose[3:6]))))
        print("  move to " + self.format_pose(poseStamped))
        (success, _, current_pose) \
            = self.go_to_pose_goal(
                self._robot_name, poseStamped, self._speed,
                end_effector_link=self._robot_effector_frame,
                move_lin=True)
        print("  reached " + self.format_pose(current_pose))
        return success

    def move_to(self, pose, keypose_num, subpose_num):
        success = self.move(pose)
        if not success:
            return False

        if self.take_sample:
            try:
                rospy.sleep(1)  # Wait for the robot to settle.
                self.continuous_shot(self._camera_name, True)
                rospy.sleep(self._sleep_time)
                res = self.take_sample()
                self.continuous_shot(self._camera_name, False)

                n = len(self.get_sample_list().cMo)
                print("  {} samples taken: {}").format(n, res.message)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                success = False

        return success

    def move_to_subposes(self, pose, keypose_num):
        subpose = copy.copy(pose)
        roll = subpose[3]
        for i in range(3):
            print("\n--- Subpose [{}/5]: Try! ---".format(i + 1))
            if self.move_to(subpose, keypose_num, i + 1):
                print("--- Subpose [{}/5]: Succeeded. ---".format(i + 1))
            else:
                print("--- Subpose [{}/5]: Failed. ---".format(i + 1))
            subpose[3] -= 30

        subpose[3] = roll - 30
        subpose[4] += 15

        for i in range(2):
            print("\n--- Subpose [{}/5]: Try! ---".format(i + 4))
            if self.move_to(subpose, keypose_num, i + 4):
                print("--- Subpose [{}/5]: Succeeded. ---".format(i + 4))
            else:
                print("--- Subpose [{}/5]: Failed. ---".format(i + 4))
            subpose[4] -= 30

    def run(self):
        self.continuous_shot(self._camera_name, False)

        if self.reset:
            self.reset()

        # Reset pose
        self.go_home()
        self.move(self._initpose)

        # Collect samples over pre-defined poses
        keyposes = self._keyposes
        for i, keypose in enumerate(keyposes, 1):
            print("\n*** Keypose [{}/{}]: Try! ***"
                  .format(i, len(keyposes)))
            if self.is_eye_on_hand():
                self.move_to(keypose, i, 1)
            else:
                self.move_to_subposes(keypose, i)
            print("*** Keypose [{}/{}]: Completed. ***"
                  .format(i, len(keyposes)))

        if self.compute_calibration:
            res = self.compute_calibration()
            print(res.message)
            res = self.save_calibration()
            print(res.message)

        self.go_home()


######################################################################
#  global functions                                                  #
######################################################################
if __name__ == '__main__':
    rospy.init_node("~")

    calibrate = HandEyeCalibrationRoutines()

    while not rospy.is_shutdown():
        if raw_input(">> ") == "q":
            break
        calibrate.run()

    sys.exit()
