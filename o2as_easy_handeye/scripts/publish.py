#!/usr/bin/env python2

import rospy
from tf import TransformBroadcaster, TransformListener, TransformerROS, transformations as tfs
from geometry_msgs.msg import Transform
from easy_handeye.handeye_calibration import HandeyeCalibration

rospy.init_node('o2as_handeye_calibration_publisher')
while rospy.get_time() == 0.0:
    pass

calib = HandeyeCalibration()
calib.from_file()

if calib.eye_on_hand:
    overriding_robot_effector_frame = rospy.get_param('robot_effector_frame')
    if overriding_robot_effector_frame != "":
        calib.transformation.header.frame_id = overriding_robot_effector_frame
else:
    overriding_robot_base_frame = rospy.get_param('robot_base_frame')
    if overriding_robot_base_frame != "":
        calib.transformation.header.frame_id = overriding_robot_base_frame
overriding_tracking_base_frame = rospy.get_param('tracking_base_frame')
if overriding_tracking_base_frame != "":
    calib.transformation.child_frame_id = overriding_tracking_base_frame

rospy.loginfo('loading calibration parameters into namespace {}'.format(rospy.get_namespace()))
calib.to_parameters()

result_tf   = calib.transformation.transform
trns        = result_tf.translation.x, result_tf.translation.y, \
              result_tf.translation.z
rot         = result_tf.rotation.x, result_tf.rotation.y, \
              result_tf.rotation.z, result_tf.rotation.w
orig        = calib.transformation.header.frame_id  # tool or base link
dest        = calib.transformation.child_frame_id   # tracking_base_frame

camera_base_frame    = rospy.get_param('camera_base_frame')
camera_optical_frame = rospy.get_param('camera_optical_frame')
listener             = TransformListener()
now                  = rospy.Time.now()
listener.waitForTransform(camera_optical_frame,
                          camera_base_frame, now, rospy.Duration(10))
xyz, rpy             = listener.lookupTransform(camera_optical_frame,
                                                camera_base_frame, now)
body                 = rospy.get_param('camera_body_frame')

rospy.loginfo('publishing transformation ' + orig + ' -> ' + dest + ':\n'
              + str((trns, rot)))

broad = TransformBroadcaster()
rate  = rospy.Rate(50)

while not rospy.is_shutdown():
    now = rospy.Time.now()
    broad.sendTransform(trns, rot, now, dest, orig)  # takes ..., child, parent
    broad.sendTransform(xyz,  rpy, now, body, dest)
    rate.sleep()
