#!/usr/bin/env python2

import os
import rospy
import yaml
from tf import TransformBroadcaster, TransformListener, transformations as tfs
from geometry_msgs import msg as gmsg
from math import radians, degrees
from easy_handeye.handeye_calibration import HandeyeCalibration


#########################################################################
#  local functions                                                      #
#########################################################################
class CalibrationPublisher:
    def __init__(self, filename):
        (self._transform, self._root_frame) = self._load_transform(filename)
        self._broadcaster = TransformBroadcaster()
        self._listener    = TransformListener()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        # Get camera(optical) <- camera(body) transformation
        opt_body = self._get_transform(rospy.get_param('camera_optical_frame'),
                                       rospy.get_param('camera_body_frame'))
        bot_opt  = ((self._transform.transform.translation.x,
                     self._transform.transform.translation.y,
                     self._transform.transform.translation.z),
                    (self._transform.transform.rotation.x,
                     self._transform.transform.rotation.y,
                     self._transform.transform.rotation.z,
                     self._transform.transform.rotation.w))

        # Get robot(effector)/ground <- robot(effector/base) transformation
        root_bot = self._get_transform(self._root_frame,
                                       self._transform.header.frame_id)

        mat = tfs.concatenate_matrices(
            self._listener.fromTranslationRotation(*root_bot),
            self._listener.fromTranslationRotation(*bot_opt),
            self._listener.fromTranslationRotation(*opt_body))
        print '\n=== Estimated world/effector <- camera(body) transformation ==='
        self._print_mat(mat)

        return True

    def spin(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self._transform.header.stamp = rospy.Time.now()
            self._broadcaster.sendTransformMessage(self._transform)
            rate.sleep()

    def _load_transform(self, filename):
        with open(filename) as calib_file:
            dict = yaml.load(calib_file.read())

        eye_on_hand = dict['eye_on_hand']

        T = gmsg.TransformStamped()
        T.header.stamp    = rospy.Time.now()
        T.header.frame_id = dict['robot_effector_frame' if eye_on_hand else
                                 'robot_base_frame']
        T.child_frame_id  = rospy.get_param('tracking_base_frame')
        T.transform       = gmsg.Transform(gmsg.Vector3(
                                               dict['transformation']['x'],
                                               dict['transformation']['y'],
                                               dict['transformation']['z']),
                                           gmsg.Quaternion(
                                               dict['transformation']['qx'],
                                               dict['transformation']['qy'],
                                               dict['transformation']['qz'],
                                               dict['transformation']['qw']))

        root_frame = T.header.frame_id if eye_on_hand else 'o2as_ground'

        return (T, root_frame)

    def _get_transform(self, target_frame, source_frame):
        now = rospy.Time.now()
        self._listener.waitForTransform(target_frame, source_frame, now,
                                        rospy.Duration(10))
        return self._listener.lookupTransform(target_frame, source_frame, now)

    def _print_mat(self, mat):
        xyz = tfs.translation_from_matrix(mat)
        rpy = map(degrees, tfs.euler_from_matrix(mat))
        print('<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'.format(xyz, rpy))


#########################################################################
#  main part                                                            #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('o2as_handeye_calibration_publisher')

    while rospy.get_time() == 0.0:
        pass

    filename = os.path.expanduser('~/.ros/easy_handeye/') \
             + rospy.get_namespace().rstrip('/').split('/')[-1] + '.yaml'

    with CalibrationPublisher(filename) as cp:
        cp.spin()
