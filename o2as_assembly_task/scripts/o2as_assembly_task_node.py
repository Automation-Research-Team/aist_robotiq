#!/usr/bin/env python

import rospy
from o2as_assembly_task import AssemblyTask

#LOG_LEVEL = log_level=rospy.DEBUG
LOG_LEVEL = log_level=rospy.INFO

if __name__ == "__main__":
    rospy.init_node('o2as_assembly_task', anonymous=True, log_level=LOG_LEVEL)
    assembly_task = AssemblyTask()
    assembly_task.run(product_count = 10000)
