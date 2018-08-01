# Introduction

This node contains:

1) A launch file to connect the physical robots to the MoveIt planner
2) Examples of how to use MoveIt

# How to start up the physical robots

    ```
    roslaunch o2as_moveit_planner connect_real_robots.launch
    roslaunch o2as_moveit_config o2as_planning_execution.launch
    ```

Don't forget to use the correct network and set the robot IPs correctly. The IPs are:
    
    a_bot (UR5, left): 192.168.0.43  
    b_bot (UR5, left): 192.168.0.42  
    c_bot (UR3, left): 192.168.0.41  
    Robotiq gripper (UR3, left): 192.168.0.40  

# How to use the examples in simulation

Start up Gazebo and MoveIt as described in the Readme of o2as_moveit_config. Then run the example launch files of this package.

The examples are mostly in C++ because the MoveIt interface in Python is not completely up to date.


# FAQ

### How to get the correct orientation for the end effector?

One way to do it:  

- Move to the desired pose in Rviz
- Under Planning Scene/Scene Robot/Links, look up a_bot_robotiq_85_tip_link (or whichever link you need)
- Enter the orientation into https://quaternions.online , obtain the Euler Angles in ZYX-order
- Enter them into this command to make it pretty: `tf::createQuaternionMsgFromRollPitchYaw(0, M_PI/4, M_PI);`

