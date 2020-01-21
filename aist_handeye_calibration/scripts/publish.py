#!/usr/bin/env python2

import os
import rospy
import yaml
from tf import TransformBroadcaster, TransformListener, transformations as tfs
from geometry_msgs import msg as gmsg
from math import radians, degrees

#########################################################################
#  local functions                                                      #
#########################################################################
class CalibrationPublisher:
    def __init__(self, filename):
        (self._X, self._Y, self._root_frame) = self._load_transforms(filename)
        self._broadcaster = TransformBroadcaster()
        self._listener    = TransformListener()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        # Get camera(optical) <- camera(body) transformation
        opt_body = self._get_transform(rospy.get_param('camera_optical_frame'),
                                       rospy.get_param('camera_body_frame'))
        bot_opt  = ((self._X.transform.translation.x,
                     self._X.transform.translation.y,
                     self._X.transform.translation.z),
                    (self._X.transform.rotation.x,
                     self._X.transform.rotation.y,
                     self._X.transform.rotation.z,
                     self._X.transform.rotation.w))

        # Get robot(effector)/ground <- robot(effector/base) transformation
        root_bot = self._get_transform(self._root_frame,
                                       self._X.header.frame_id)

        mat = tfs.concatenate_matrices(
            self._listener.fromTranslationRotation(*root_bot),
            self._listener.fromTranslationRotation(*bot_opt),
            self._listener.fromTranslationRotation(*opt_body))
        print '\n=== Estimated effector/world <- camera(body) transformation ==='
        self._print_mat(mat)

        print '\n=== Estimated world/effector <- marker transformation ==='
        self._print_transform(self._Y.transform)

        return True

    def spin(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self._X.header.stamp = now
            self._broadcaster.sendTransformMessage(self._X)
            self._Y.header.stamp = now
            self._broadcaster.sendTransformMessage(self._Y)
            rate.sleep()

    def _load_transforms(self, filename):
        with open(filename) as calib_file:
            dict = yaml.load(calib_file.read())

        eye_on_hand = dict['eye_on_hand']

        X = gmsg.TransformStamped()
        X.header.stamp    = rospy.Time.now()
        X.header.frame_id = dict['robot_effector_frame' if eye_on_hand else
                                 'robot_base_frame']
        X.child_frame_id  = rospy.get_param('tracking_base_frame')
        X.transform       = gmsg.Transform(
                                gmsg.Vector3(
                                    dict['transformation']['x'],
                                    dict['transformation']['y'],
                                    dict['transformation']['z']),
                                gmsg.Quaternion(
                                    dict['transformation']['qx'],
                                    dict['transformation']['qy'],
                                    dict['transformation']['qz'],
                                    dict['transformation']['qw']))
        Y = gmsg.TransformStamped()
        Y.header.frame_id = dict['robot_base_frame' if eye_on_hand else
                                 'robot_effector_frame']
        Y.child_frame_id  = rospy.get_param('tracking_marker_frame')
        Y.transform       = gmsg.Transform(
                                gmsg.Vector3(
                                    dict['another_transformation']['x'],
                                    dict['another_transformation']['y'],
                                    dict['another_transformation']['z']),
                                gmsg.Quaternion(
                                    dict['another_transformation']['qx'],
                                    dict['another_transformation']['qy'],
                                    dict['another_transformation']['qz'],
                                    dict['another_transformation']['qw']))

        root_frame = X.header.frame_id if eye_on_hand else 'ground'

        return (X, Y, root_frame)

    def _get_transform(self, target_frame, source_frame):
        now = rospy.Time.now()
        self._listener.waitForTransform(target_frame, source_frame, now,
                                        rospy.Duration(10))
        return self._listener.lookupTransform(target_frame, source_frame, now)

    def _print_mat(self, mat):
        xyz = tfs.translation_from_matrix(mat)
        rpy = map(degrees, tfs.euler_from_matrix(mat))
        print('<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'.format(xyz, rpy))


    def _print_transform(self, transform):
        xyz = (transform.translation.x,
               transform.translation.y, transform.translation.z)
        rpy = map(degrees, tfs.euler_from_quaternion((transform.rotation.x,
                                                      transform.rotation.y,
                                                      transform.rotation.z,
                                                      transform.rotation.w)))
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
