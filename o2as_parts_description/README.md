# o2as_parts_description
This package includes URDF and mesh models of the parts used in WRC2018. The 
models can be used for simulation. You can check the parts are correctly 
spawned in the simulation environment by following the steps below.

1. Launch docker container
    ```bash
    $ sh RUN-DOCKER-CONTAINER.sh
    ```
2. Launch simulated environment in Gazebo
    ```bash
    # In the container
    $ source ~/catkin_ws/devel/setup.bash
    $ roslaunch o2as_gazebo o2as_gazebo.launch
    ```
3. Run the script for test
    ```bash
    # 4 is the number of a part
    $ rosrun o2as_parts_description test_spawn_parts.py 4
    ```
