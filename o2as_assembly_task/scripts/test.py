#!/usr/bin/env python

import rospy
from o2as_assembly_task import PipelineTest

LOG_LEVEL = log_level=rospy.DEBUG

if __name__ == "__main__":
    rospy.init_node('o2as_assembly_task', anonymous=True, log_level=LOG_LEVEL)
    test = PipelineTest()
    test.run()
