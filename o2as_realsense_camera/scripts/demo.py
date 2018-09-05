#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from o2as_realsense_camera.client import *

if __name__ == "__main__":
    rospy.init_node('realsense_camera_demo', anonymous=True, log_level=rospy.DEBUG)

    # connect to the camera server using client class
    camera_node_name = rospy.get_param("~camera_node_name")
    camera = RealSenseCameraClient(camera_node_name)
    bridge = CvBridge()

    cv2.namedWindow('color_image', cv2.WINDOW_NORMAL)
    cv2.namedWindow('depth_image', cv2.WINDOW_NORMAL)
    
    try:
        while not rospy.is_shutdown():
            resp = camera.get_frame(dump=False, publish=True)
            resp = camera.get_frame(dump=True, publish=True)

            # display captured frame
            cv_color_image = bridge.imgmsg_to_cv2(resp.color_image, desired_encoding="passthrough")
            cv_depth_image = bridge.imgmsg_to_cv2(resp.depth_image, desired_encoding="passthrough")
            rospy.logdebug("image size: " + str(cv_color_image.shape))
            cv2.imshow('color_image', cv_color_image)
            cv2.imshow('depth_image', cv_depth_image)
            cv2.waitKey(1)

    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s" % e)

    cv2.destroyAllWindows()
