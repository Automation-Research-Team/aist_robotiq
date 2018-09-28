#! /usr/bin/env python
from time import sleep, time
import rospy
from std_msgs.msg import String, Int32
from o2as_debug_monitor.msg import PressureSensoState

if __name__ == "__main__":
  # Initialize the ROS node
  rospy.init_node("test_debug_monitor")

  # Initialize publishers
  pub1 = rospy.Publisher("/o2as_state/kitting_set_id", Int32, queue_size=1)
  pub2 = rospy.Publisher("/o2as_state", String, queue_size=1)
  pub3 = rospy.Publisher("/o2as_fastening_tools/screw_suctioned",
                         PressureSensoState, queue_size=18) # see issue 133

  # Wait until debug monitor launch
  sleep(2);

  # Loop over set lists
  for i in range(1, 4):
    # Start round
    msg = Int32()
    msg.data = i
    pub1.publish(msg)

    # Loop over subtasks
    for j in range(10):
      sleep(1)
      msg = String()
      msg.data = str(time())
      pub2.publish(msg)
