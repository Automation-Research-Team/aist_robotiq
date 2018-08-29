import rospy

def ros_service_proxy(service_name, service_type):
    proxy = None
    try:
        rospy.logdebug("wait for service %s", service_name)
        rospy.wait_for_service(service_name)
        proxy = rospy.ServiceProxy(service_name, service_type)
    except rospy.ServiceException as e:
        rospy.logerr("service error: %s", str(e)) 
    return proxy
