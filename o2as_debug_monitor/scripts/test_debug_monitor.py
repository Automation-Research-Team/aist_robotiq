#! /usr/bin/env python
from time import sleep, time
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
  # Initialize the ROS node
  rospy.init_node("test_debug_monitor")

  # Initialize a publisher
  pub = rospy.Publisher("/o2as_state", String, queue_size=1)

  # Loop
  while True:
    sleep(1)
    msg = String()
    msg.data = str(time())
    pub.publish(msg)
