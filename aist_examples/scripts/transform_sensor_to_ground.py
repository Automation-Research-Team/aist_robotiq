#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg

if __name__ == "__main__":

    rospy.init_node('transform_sensor_to_ground', anonymous=True)
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        x = raw_input('x:')
        y = raw_input('y:')
        z = raw_input('z:')
        if x == '' or y == '' or z == '':
            break
        point = geometry_msgs.msg.PointStamped()
        point.header.frame_id = 'a_phoxi_m_sensor'
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)

        point_world = listener.transformPoint('o2as_ground', point)
        rospy.loginfo('transformed point:')
        rospy.loginfo(point_world)
