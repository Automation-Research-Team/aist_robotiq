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

one robot
$ roslaunch aist_iiwa_gazebo iiwa_gazebo.launch trajectory:=false

two robots
$ roslaunch aist_iiwa_gazebo iiwa_gazebo.launch trajectory:=false robot_2:=true

two robots, end_effector(bh282)
$ roslaunch aist_iiwa_gazebo iiwa_gazebo.launch trajectory:=false robot_2:=true end_effector:=bh282

--------------------------------------------------------------------------------
[ moveit ]

2 robots, end_effector(bh282)
$ roslaunch aist_iiwa_moveit moveit_planning_execution.launch end_effector:=bh282

--------------------------------------------------------------------------------
[ pick & place, pose]

pick & place, end_effector(bh282)
$ rosrun aist_iiwa_pick_place pick.py

pick & place, end_effector(robotiq85grippe)
$ rosrun aist_iiwa_pick_place pick.py a_iiwa r85g b_iiwa r85g iiwa_two_robots_groups r85g

pose (a_iiwa, b_iiwa)
$ rosrun aist_iiwa_pick_place pose.py

pose (a_iiwa_groups, b_iiwa_groups)
$ rosrun aist_iiwa_pick_place pose.py a_iiwa_groups a_iiwa_link_ee b_iiwa_groups b_iiwa_link_ee

================================================================================

$ roslaunch aist_routines connect_robots.launch config:=iiwa sim:=true

aist_gazebo/launch/iiwa_gazebo.launch
aist_routines/launch/iiwa_connect_robots.launch
  enable two robots
    <arg name="robot_2" default="true"/>
  only one robot
    <arg name="robot_2" default="false"/>

(a_iiwa)
$ rosrun aist_routines iiwa_interactive.py
(b_iiwa)
$ rosrun aist_routines iiwa_interactive.py -r b_iiwa

    pose(x, y, z, roll, pitch, yaw) を指定して動かす

        moveit>> reset
        moveit>> X
        moveit>> 0.3
        moveit>> Y
        moveit>> -0.2
        moveit>> Z
        moveit>> 2
        moveit>> go

            example:
                X,   Y,   Z, Roll, Pitch, Yaw
              0.3, 0.0, 2.0,    0,     0,   0
              0.0, 0.0, 2.1,    0,     0, 180

    object を削除
        moveit>> remove_objs

    object を表示
        moveit>> show_objs

    object に対して pose する
        moveit>> pose_obj

--------------------------------------------------------------------------------

