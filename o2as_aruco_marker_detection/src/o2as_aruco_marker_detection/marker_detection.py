import rospy
from o2as_aruco_marker_detection.srv import *

def ros_service_proxy(service_name, service_type):
  proxy = None
  try:
    rospy.logdebug("wait for service %s", service_name)
    rospy.wait_for_service(service_name)
    proxy = rospy.ServiceProxy(service_name, service_type)
  except rospy.ServiceException as e:
    rospy.logerr("service error: %s", str(e)) 
  return proxy

class MarkerDetection(object):
  def __init__(self):
    self.detect_marker_ = ros_service_proxy("/marker_detector/detect_marker", DetectMarker)

  def detect_marker(self, cloud, image, marker_id):
    res = self.detect_marker_(cloud, image, marker_id, False)
    if res.success:
      return res.marker
    return None
    