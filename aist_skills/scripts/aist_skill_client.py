#!/usr/bin/env python

from math import pi

import rospy
import geometry_msgs.msg
import tf_conversions

import o2as_msgs.srv

if __name__ == '__main__':
    rospy.init_node("aist_skills_client", anonymous=True)
    downward_orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, pi/2, 0))

    try:
        publishMarker_client = rospy.ServiceProxy('aist_skills/publishMarker', o2as_msgs.srv.publishMarker)
        while True:
            rospy.loginfo("Enter 1 to publish marker upon 10cm above workspace_center.")
            rospy.loginfo("Enter x to exit.")

            i = raw_input()
            if i == '1':
                pose = geometry_msgs.msg.PoseStamped()
                pose.header.frame_id = 'workspace_center'
                pose.pose.position.z = 0.1
                pose.pose.orientation = downward_orientation
                marker_type = "pose"
                req = o2as_msgs.srv.publishMarkerRequest()
                req.marker_pose = pose
                req.marker_type = marker_type
                publishMarker_client(req)
            elif i == 'x':
                break
        
        print("================ done!!!")
    except rospy.ROSInterruptException:
        pass

