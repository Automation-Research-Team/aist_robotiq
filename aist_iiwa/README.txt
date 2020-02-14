--------------------------------------------------------------------------------
[ end_effector ]

tool
https://github.com/SalvoVirga/iiwa_stack_examples.git

bh282
https://github.com/RobotnikAutomation/barrett_hand_common.git
https://github.com/RobotnikAutomation/barrett_hand_sim.git

robotiq85gripper
https://github.com/crigroup/robotiq.git

--------------------------------------------------------------------------------
[ gazebo ]

----- 1 robot -----
$ roslaunch aist_gazebo iiwa_gazebo.launch trajectory:=false
----- 2 robots -----
$ roslaunch aist_gazebo iiwa_gazebo.launch trajectory:=false robot_2:=true
----- 2 robots, end_effector(bh282) -----
$ roslaunch aist_gazebo iiwa_gazebo.launch trajectory:=false robot_2:=true end_effector:=bh282

--------------------------------------------------------------------------------

