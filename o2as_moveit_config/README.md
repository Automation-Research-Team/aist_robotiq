# How to use

This is still rudimentary, but you can run the 3 robots in simulation via these commands. **Do not run this package on live robots yet!**

roslaunch o2as_gazebo o2as_gazebo_3_controllers.launch
roslaunch o2as_moveit_config o2as_moveit_planning_execution.launch sim:=true

This opens gazebo and rviz in two windows. In rviz, you can:

- Select "all_bots" in the Displays box, under "Motion Planning/Planning Request/Planning Group". Then, click the tab "Planning", select "home_all" in the "Select Goal State" area, click "Update", then "Plan and Execute", and the robots will move to the default pose.
- By selecting a different Planning Group under "Motion Planning/Planning Request/Planning Group" in the Displays box, you can plan motions for different robots using the interactive marker. The robot will move in MoveIt and Gazebo when you execute the plan.


# ToDo

- Add the gripper, at least for path planning.
- Set default pose for the robots in gazebo