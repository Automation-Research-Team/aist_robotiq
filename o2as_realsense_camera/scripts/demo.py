#!/usr/bin/env python
import rospy
from o2as_realsense_camera.srv import *
from std_srvs.srv import *

if __name__ == "__main__":
    rospy.init_node('camera_client', anonymous=True)

    # get params
    pcloud_filename = rospy.get_param("~pcloud_filename")
    image_filename = rospy.get_param("~image_filename")

    # wait for service
    rospy.wait_for_service('connect')
    connect = rospy.ServiceProxy('connect', Connect)
    rospy.wait_for_service('save_frame_for_cad_matching')
    save_frame_for_cad_matching = rospy.ServiceProxy('save_frame_for_cad_matching', SaveFrameForCadMatching)

    # main loop
    r = rospy.Rate(100)
    try:
        connect()
        while not rospy.is_shutdown():
            rospy.loginfo("save frame for cad matching")
            res = save_frame_for_cad_matching(pcloud_filename, image_filename)
            r.sleep()
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
