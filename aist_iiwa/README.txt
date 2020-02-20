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

1 robot
$ roslaunch aist_iiwa_gazebo iiwa_gazebo.launch trajectory:=false

2 robots
$ roslaunch aist_iiwa_gazebo iiwa_gazebo.launch trajectory:=false robot_2:=true

2 robots, end_effector(bh282)
$ roslaunch aist_iiwa_gazebo iiwa_gazebo.launch trajectory:=false robot_2:=true end_effector:=bh282

--------------------------------------------------------------------------------
[ moveit ]

2 robots, end_effector(bh282)
$ roslaunch aist_iiwa_moveit moveit_planning_execution.launch end_effector:=bh282

================================================================================

$ roslaunch aist_routines connect_robots.launch config:=iiwa sim:=true

$ rosrun aist_routines iiwa_interactive.py -r a_iiwa
        moveit>> reset
        moveit>> X
        moveit>> 0.3
        moveit>> Y
        moveit>> -0.2
        moveit>> Z
        moveit>> 2
        moveit>> go

--------------------------------------------------------------------------------

