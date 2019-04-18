#!/usr/bin/env python2

import rospy
from tf import TransformBroadcaster, TransformListener, TransformerROS, transformations as tfs
from geometry_msgs.msg import Transform
from math import radians, degrees
from easy_handeye.handeye_calibration import HandeyeCalibration


#########################################################################
#  local functions                                                      #
#########################################################################
def get_transform(dst_frm, src_frm):
    listener = TransformListener()
    now = rospy.Time.now()
    listener.waitForTransform(dst_frm, src_frm, now, rospy.Duration(10))
    return listener.lookupTransform(dst_frm, src_frm, now)


def get_estimated_transformation():
    calib = HandeyeCalibration()
    calib.from_file()

    # Set ID of optical frame of the camera.
    calib.transformation.child_frame_id = rospy.get_param(
        'tracking_base_frame')

    # Set ID of effector/base frame of the robot.
    calib.transformation.header.frame_id = \
      rospy.get_param('robot_effector_frame') if calib.eye_on_hand else \
      rospy.get_param('robot_base_frame')

    rospy.loginfo('loading calibration parameters into namespace {}'.format(
        rospy.get_namespace()))
    calib.to_parameters()

    root_frame = calib.transformation.header.frame_id if calib.eye_on_hand else \
                 'o2as_ground'

    return calib.transformation, root_frame


def print_mat(mat):
    xyz = tfs.translation_from_matrix(mat)
    rpy = map(degrees, tfs.euler_from_matrix(mat))
    print '<origin xyz="{0[0]} {0[1]} {0[2]}" rpy="${{{1[0]}*pi/180}} ${{{1[1]}*pi/180}} ${{{1[2]}*pi/180}}"/>'.format(
        xyz, rpy)
    q = tfs.quaternion_from_matrix(mat)
    print(xyz, q)


def print_camera_pose(root_bot, bot_opt, opt_body):
    transformer = TransformerROS()

    # Print ground/effector <- camera(body) transformation
    mat = tfs.concatenate_matrices(
        transformer.fromTranslationRotation(*root_bot),
        transformer.fromTranslationRotation(*bot_opt),
        transformer.fromTranslationRotation(*opt_body))
    print '\n=== Estimated ground/effector <- camera(body) transformation ==='
    print_mat(mat)


#########################################################################
#  main part                                                            #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('o2as_handeye_calibration_publisher')

    while rospy.get_time() == 0.0:
        pass

    # Get camera(optical) <- camera(body) transformation
    opt_body = get_transform(rospy.get_param('camera_optical_frame'),
                             rospy.get_param('camera_body_frame'))

    # Estimated robot(effector/base) <- camera(optical) transformation
    transformation, root_frame = get_estimated_transformation()
    bot_opt = ((transformation.transform.translation.x,
                transformation.transform.translation.y,
                transformation.transform.translation.z),
               (transformation.transform.rotation.x,
                transformation.transform.rotation.y,
                transformation.transform.rotation.z,
                transformation.transform.rotation.w))
    bot_frame = transformation.header.frame_id  # real effector/base frame
    opt_frame = transformation.child_frame_id  # real camera(optical)_frame

    # Get robot(effector)/ground <- robot(effector/base) transformation
    root_bot = get_transform(root_frame, bot_frame)

    try:
        broad = TransformBroadcaster()
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            now = rospy.Time.now()  # child -> # parent
            broad.sendTransform(bot_opt[0], bot_opt[1], now, opt_frame,
                                bot_frame)
            rate.sleep()

        print_camera_pose(root_bot, bot_opt, opt_body)

    except rospy.ROSInterruptException:
        print_camera_pose(root_bot, bot_opt, opt_body)
