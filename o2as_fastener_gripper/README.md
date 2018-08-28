# Introduction
Control the XL320 with ROS.

It is implemented by Action Server.

Turn in the 'CCW' direction.
When the resistance increases and the speed becomes 0, the rotation stops.

# Initial Setup
* Dynamixel SDK

Install the Dynamixel SDK to control XL320.

The following is the part described in Dockerfile.
```bash
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-dynamixel-sdk \
```
This package controls XL320, so 'dynamixel-workbench' and 'dynamixel-workbench-msgs' are dependent packages.

* U2D2

For enabling the connection of the U2D2 controllers to a named port, please refer [here](https://gitlab.com/o2as/ur-o2as/blob/develop/udev_rules.md).

Five U2D2 controllers are used in this package. The parameters are set in `demo.launch`.
Please set the maximum number of connections and connection destination.

* ID of XL-320

The motor id needs to be set to move a motor. The ID has to be unique for each Dynamixel motor.

Execute 'motor_id_set.launch'.
The information of a connected motor is displayed.
Through the input screen of the ID setting node, you can set a new ID.

Take care not to have the same IDs for Dynamixels connected to the same controller. You may face communication issues or may not be able to search when IDs overlap.

Specifically, refer to [here](http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm#Actuator_Address_03).

* config/gripper.yaml

Please set a name for the set motor ID.
Item names are 'name' and 'motor_id'.
You will set parameters with that name.

# How to use
To use the action, use the gripper name and rotation speed.

The gripper name is set with gripper.yaml in the config folder.

Speed can be specified as follows.

If a value in the range of 0~1023 is used, it is stopped by setting to 0 while rotating to CCW direction.

If a value in the range of 1024~2047 is used, it is stopped by setting to 1024 while rotating to CW direction.

Specifically, please refer to [here](http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm#Actuator_Address_03).

As in demo.launch, set parameter.

'max_access' is the number of U2D2 connections.

'access_no' is set as many set by 'max_access'. 
The value uses the connection name of U2D2.

'conf_dir' sets the path of the config folder.

'parts' sets the yaml name with gripper information.


* Example of use 

Appoint gripper name and rotary speed in 'fastener_gripper_action_client.py'.

Please connect U2D2 and XL320 to PC and run 'demo.launch'.

Two XL320 will be used this time.
ID is 1,2 respectively.

I think that the following operation is possible.

* 1. Turn on the torque of XL-320 and rotate clockwise (wheel mode)
* 2. Stop rotation of the motor with fingers, generate load (assuming screws close)
* 3. When the load is detected (rotation speed = 0), the torque of XL-320 is turned off and the rotation stop
* 4. To next motor id .


