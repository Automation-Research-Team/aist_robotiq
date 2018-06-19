# How to use

This is still rudimentary, but you can run the 3 robots in simulation via these commands. **Do not run this package on live robots yet!**

roslaunch o2as_gazebo o2as_gazebo_3_controllers.launch
roslaunch o2as_moveit_config o2as_moveit_planning_execution.launch sim:=true

This opens gazebo and rviz in two windows. In rviz:

- In the "Displays" box in the top left, set the fixed frame to "world" instead of "map"
- Click "Add" and add the MotionPlanning item
- Expand the MotionPlanning item in the "Displays" box, then expand the sub-item "Planning Request" and set the Interactive Marker Size to 0.5 
- By selecting a different the Planning Group here, you can direct different robots
- In the MotionPlanning box in the bottom left of the window, click on "Unspecified" under "OMPL" and select "RRTkConfigDefault", or any other planner.
- Click the tab "Planning", move the robot to another position via the interactive marker, and click "Plan and Execute". The robot will move in MoveIt and Gazebo.


# ToDo

- Set up rviz so that it automatically displays the MotionPlanning and interactive marker
- Add the scene(s) (parts trays, bins, assembly areas)
- Add the gripper, at least for path planning.