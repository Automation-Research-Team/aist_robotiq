#!/usr/bin/env python
import socket
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import WrenchStamped

def WrenchPublish():
    pub=rospy.Publisher("ft300wrench",WrenchStamped, queue_size=100)
    rospy.init_node('FT300WrenchData', anonymous=True)
    rate=rospy.Rate(100)
    wren=WrenchStamped()
    HOST = "10.66.171.51" # The remote host
    PORT = 63351 # The same port as used by the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))
    while not rospy.is_shutdown():
        data = s.recv(1024)
        data = data.replace("(","")
        # data = data.replace(",","\t")
        # data = data.replace(")","\n")
        data = data.replace(")","")
        datasplit=data.split(",")
        datasplit= [float(i) for i in datasplit]
        wren.wrench.force.x=datasplit[0]
        wren.wrench.force.y=datasplit[1]
        wren.wrench.force.z=datasplit[2]
        wren.wrench.torque.x=datasplit[3]
        wren.wrench.torque.y=datasplit[4]
        wren.wrench.torque.z=datasplit[5]
        wren.header=Header()
        wren.header.stamp=rospy.Time.now()
        rospy.loginfo(wren)
        pub.publish(wren)
        rate.sleep()
    s.close

if __name__ == '__main__':
    try:
        WrenchPublish()
    except rospy.ROSInterruptException:
        pass
