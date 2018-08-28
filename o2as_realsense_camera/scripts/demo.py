#!/usr/bin/env python
import rospy
from o2as_realsense_camera.client import *

if __name__ == "__main__":
    rospy.init_node('camera_client', anonymous=True)

    # get params
    setting_filename = rospy.get_param("~setting_filename")
    pcloud_filename = rospy.get_param("~pcloud_filename")
    image_filename = rospy.get_param("~image_filename")

    # capture image from camera and save them to file continuously
    camera = RealSenseCameraClient()
    r = rospy.Rate(60)
    try:
        camera.connect()
        camera.save_camera_info(setting_filename)
        while not rospy.is_shutdown():
            rospy.loginfo("save frame for cad matching")
            res = camera.save_frame_for_cad_matching(pcloud_filename, image_filename)
            r.sleep()
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)
