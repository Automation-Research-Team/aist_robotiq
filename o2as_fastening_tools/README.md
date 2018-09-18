# Introduction
This package controls our fastening tools which are driven by an XL320 motor. 

## Available ROS services

```
o2as_fastening_tools/fastener_gripper_control_action
```

Please read 'How to use' for details. 

# Initial Setup
## Dynamixel SDK

The Dynamixel SDK needs to be installed to control the XL320 motors.

This package depends on 'dynamixel-workbench' and 'dynamixel-workbench-msgs'. If you have any trouble building, try this:
```bash
apt-get update && apt-get install -y --no-install-recommends ros-kinetic-dynamixel-sdk
```

## U2D2

For enabling the connection of the U2D2 controllers to a named port, please refer [here](https://gitlab.com/o2as/ur-o2as/blob/develop/udev_rules.md).

## ID of XL-320

The motor id needs to be set to move a motor. The ID has to be unique for each Dynamixel motor.

Execute 'motor_id_set.launch'.
The information of a connected motor is displayed.
Through the input screen of the ID setting node, you can set a new ID.

Take care not to have the same IDs for Dynamixels connected to the same controller. You may face communication issues or may not be able to search when IDs overlap.

Specifically, refer to [here](http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm#Actuator_Address_03).

Enter the names of each motor ID and gripper in this file: config/gripper.yaml

# How to use
## About action service

load gripper info file. (fastening_tools.yaml)

To use the action service, use the gripper name and rotation speed or rotation time and drirection of rotation.
The return value is the boolean type.
The feedback value is current the rotation speed of uint32 type.

- gripper name : 
The gripper name is set with gripper.yaml in the config folder.
In this file, the motor are listed.
If you want to add a new motor, please add it to this file.
However, please do not duplicate the motor name and id.

- drirection : 
The drirection of rotation can be either 'tighten' or 'loosen'.
The motion is different in each direction.

- rotation speed : 
'tighten' will rotate until the motor stops rotating.
The rotation speed is mandatory for 'tighten'.
The rotation speed can be between 0 and 1023.
The rotation speed is uint32 type.

- drirection of rotation : 
'loosen' will rotate for the specified time.
The rotation time is mandatory for 'loosen'.
The rotation time is float32 type.
In this case, there is no feedback.

## About the actual launch file

As in fastening_tools.launch, set parameter.

'max_access' is the number of U2D2 connections.

'access_no' is set as many set by 'max_access'. 
The value uses the connection name of U2D2.

'conf_dir' sets the path of the config folder.

'fastening_tools' sets the yaml name with gripper information.


## Example of use 

Appoint gripper name and rotary speed in 'fastener_gripper_action_client.py'.

Please connect U2D2 and XL320 to PC and run 'demo.launch'.

Two XL320 will be used this time.
ID is 1,2 respectively.

I think that the following operation is possible.

- 1. Turn on the torque of XL-320 and rotate clockwise (wheel mode)
- 2. Stop rotation of the motor with fingers, generate load.
- 3. When the load is detected (rotation speed = 0), the torque of XL-320 is turned off and the rotation stop
- 4. To next motor id .


