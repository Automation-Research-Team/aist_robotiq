#!/usr/bin/env python
import rospy
from o2as_realsense_camera.srv import *
from std_srvs.srv import *

if __name__ == "__main__":
    rospy.init_node('client', anonymous=True)

    pcloud_filename = rospy.get_param("~pcloud_filename")
    image_filename = rospy.get_param("~image_filename")

    rospy.wait_for_service('/camera1/get_frame')
    get_frame = rospy.ServiceProxy('/camera1/get_frame', GetFrame)
    rospy.wait_for_service('/camera1/save_frame_for_cad_matching')
    save_frame_for_cad_matching = rospy.ServiceProxy('/camera1/save_frame_for_cad_matching', SaveFrameForCadMatching)

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            #res = get_frame(-1)
            res = save_frame_for_cad_matching(pcloud_filename, image_filename)
            rospy.loginfo("")
            r.sleep()
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
