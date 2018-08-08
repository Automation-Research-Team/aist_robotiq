#!/usr/bin/env python

import rospy
import moveit_msgs.msg
import std_msgs.msg

import os, rospkg

def main():
  rospy.init_node('urscript_sending_example_node',
                    anonymous=True)
  self.pub = rospy.Publisher("/b_bot/URscript", std_msgs.msg.String, queue_size=1)
  
  # Constructs a program to send to the robot.
  complete_program = ""
  
  # --- Example to read in a file
#   program_template = open(os.path.join(self.rospack.get_path("o2as_exampleso2as_moveit_planner"), "src", "robotiq_urscript_move.ur"), 'rb')
#   program_line = program_template.read(1024)
#   while program_line:
#       #complete_program += program_line.decode()
#       complete_program += program_line
#       program_line = program_template.read(1024)
  # ---
  
  # Add lines manually
  complete_program += "def move_forward_10cm(): \n"
  complete_program += "  start_pos=get_forward_kin() \n"
  complete_program += "  target_pos=pose_trans(start_pos,p[0.0, 0.0, 100.0, 0.0, 0.0, 0.0]) \n"
  complete_program += "  movel(pose_trans(p[0.0,0.0,0.0,0.0,0.0,0.0], target_pos), a=1.2, v=0.25) \n"
  complete_program += "end \n"

  rospy.loginfo("Sending command.")
  
  # Wrap as std_msgs/String
  program_msg = std_msgs.msg.String()
  program_msg.data = complete_program
  
  self.pub.publish(program_msg)

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    exit(1)
  except KeyboardInterrupt:
    exit(0)
