# Introduction

This node contains:

1) A launch file to connect the physical robots to 
2) Examples 

# How to start up the physical robots

    ```
    roslaunch o2as_moveit_planner connect_physical_robots.launch
    roslaunch o2as_moveit_config o2as_planning_execution.launch
    ```

Don't forget to use the correct network and set the robot IPs correctly.

# How to use the examples in simulation

Start up Gazebo and MoveIt as described in the Readme of o2as_moveit_config. Then run the example launch files of this package.

The examples are mostly in C++ because the MoveIt interface in Python is not completely up to date.