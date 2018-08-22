#!/usr/bin/env python

import rospy
from aist_kitting.srv import *
from std_srvs.srv import Trigger
from o2as_phoxi_camera.srv import SetInt
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def handle_get_image(req):
    if req.camera == "phoxi":
        rospy.wait_for_service("o2as_phoxi_camera/start_acquisition")
        start = rospy.ServiceProxy("o2as_phoxi_camera/start_acquisition", Trigger)
        res_start = start()

        trigger = rospy.ServiceProxy("o2as_phoxi_camera/trigger_frame", Trigger)
        res_trigger = trigger()
        while not res_trigger.success
            res_trigger = trigger()
        get_frame = rospy.ServiceProxy("o2as_phoxi_camera/get_image", SetInt)
        res_get_frame = get_frame(0)
        msg_depth = rospy.wait_for_message("o2as_phoxi_camera/depth_map", Image, timeout=None)
        bridge = CvBridge()
        bridge.imgmsg_to_cv2(msg_depth, )
        
        stop = rospy.ServiceProxy("o2as_phoxi_camera/stop_acquisition", Trigger)
        res_stop = stop()

def aist_kitting_server():
    rospy.loginfo("Start aist_kiting_server")
    rospy.init_node("aist_kitting_server", anonymous=True)

    get_image = rospy.Service("get_image", GetImage, handle_get_image)

    rospy.spin()

if __name__ == '__main__':
    try:
        aist_kitting_server()
    except rospy.ROSInterruptException as e:
        rospy.logerr(e.message)
        exit(1)
    except KeyboardInterrupt:
        exit(0)
