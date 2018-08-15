#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used
import o2as_msgs.msg

def PrecisionGripperActionClient():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('precision_gripper_action', o2as_msgs.msg.PrecisionGripperCommandAction)

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()
    
    # Creates a goal to send to the action server.
    goal = o2as_msgs.msg.PrecisionGripperCommandGoal()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can publish and subscribe over ROS.
        rospy.init_node('precision_gripper_action_client')
        result = PrecisionGripperActionClient()
        rospy.loginfo(result)
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion", file=sys.stderr)