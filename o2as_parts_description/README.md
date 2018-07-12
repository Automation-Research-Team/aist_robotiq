# o2as_parts_description

This package includes URDF and mesh models of the parts used in WRC2018. The 
models can be used for simulation. You can check the parts are correctly 
spawned in the simulation environment by following the steps below.

1. Launch simulated environment in Gazebo
    ```bash
    # In the container
    source ~/catkin_ws/devel/setup.bash
    roslaunch o2as_gazebo o2as_gazebo.launch
    ```
2. Run the script for test
    ```bash
    # 7 is the part ID, 5 is the number of parts to spawn, and 1 is the bin ID
    rosrun o2as_parts_description test_spawn_parts.py 7 5 1
    rosrun o2as_parts_description test_spawn_parts.py 8 10 2
    rosrun o2as_parts_description test_spawn_parts.py 9 50 3
    rosrun o2as_parts_description test_spawn_parts.py 16 50 4
    ```
