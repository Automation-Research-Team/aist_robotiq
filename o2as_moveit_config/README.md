# How to use

This is still rudimentary, but you can run the 3 robots in simulation via these commands. **Do not run this package on live robots yet!**

roslaunch o2as_gazebo o2as_gazebo_3_controllers.launch
roslaunch o2as_moveit_config o2as_moveit_planning_execution.launch sim:=true

This opens gazebo and rviz in two windows. In rviz, you can:

- In the MotionPlanning box in the bottom left of the window, click on "Unspecified" under "OMPL" and select "RRTkConfigDefault", or any other planner.
- Click the tab "Planning", move the robot to another position via the interactive marker, and click "Plan and Execute". The robot will move in MoveIt and Gazebo.
- By selecting a different Planning Group under "Motion Planning/Planning Request" in the Displays box, you can plan motions for different robots.


# ToDo

- Add the scene(s) (parts trays, bins, assembly areas)
- Add the gripper, at least for path planning.