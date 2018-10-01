# Introduction
This package controls our fastening tools which are driven by an XL320 motor. 
It also controls execution and judgment of screw suction.

## Available ROS services
There are two action services.

```
o2as_fastening_tools/fastener_gripper_control_action
o2as_fastening_tools/screw_control_action
```

There is also one Publish.
```
o2as_fastening_tools/screw_suctioned
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
### fastener_gripper_control_action

The action topic is "o2as_fastening_tools/fastening_gripper_control_action" and the action is defined in o2as_msgs.

load gripper info file. (fastening_tools.yaml)

To use the action service, use "fastening_tool_name" and "speed" or "duration" and "direction".
The return value is the boolean type.
The feedback value is current the rotation speed of uint32 type.

- "fastening_tool_name" : 
The gripper name is set with gripper.yaml in the config folder.
In this file, the motor are listed.
If you want to add a new motor, please add it to this file.
The gripper name can be any string.
However, please do not duplicate the motor name and id.

- "direction" : 
The valid strings for "direction" is "tighten" or "loosen".
The motion is different in each valid strings.

- "speed" : 
When the "direction" is "tighten" mode, this will rotate until the motor stops rotating.
When the "direction" is "loosen" mode, this will rotate until the "duration" seconds.
The "speed" is mandatory for "tighten" and "loosen" mode.
The "speed" can be between 0 and 1023.
The "speed" is uint32 type.

- "duration" : 
When the "direction" is "loosen" mode, this will rotate for the specified time.
The "duration" is mandatory for "loosen" mode.
The "duration" can be in seconds.
The "duration" is float32 type. (ex. "duration" = 1.0 : one second) 
In this mode, there is no feedback.

### screw_control_action

The action topic is "o2as_fastening_tools/screw_control_action" and the action is defined in o2as_msgs.

load ur_control info file. (screw_control.yaml)

This action is used when you want to execute suction on/off.
If this action is insert screw or trying to break vacuum, if it does not get the result after 5 seconds, it will return 'False' and exit.

To use the action service, use "fastening_tool_name" and "screw_control" and "switch".
The return value is the boolean type.
The feedback value is current the digital_input of ur_controller states of boolean type.

- "fastening_tool_name" : 
The gripper name is set with screw_control.yaml in the config folder.
In this file, the name of screw fastening tool and other info are listed.
For details, I think that you can understand by looking at screw_control.yaml

- "screw_control" :
The valid strings for "screw_control" is "Insert" or "Remove".

- "switch" :
This value is used to turn digital output pin of ur_control on and off.
True is on. False is off.

This action service uses the service 'b_bot_controller/ur_driver/set_io' of the sub module ur_modern_driver.
Please be careful if you use 'b_bot_controller/ur_driver/set_io' as an alias.

## About Publish screw_suctioned

This topic in screw_controller.py.
This topic is publish suction success for screw tools.
The msg file is PressureSensoState.msg.

Publish of pin number of digital input pf ur_control and its state.

This topic uses the publish 'ur_driver/io_states' of the sub module ur_modern_driver.
Please be careful if you use 'ur_driver/io_states' as an alias.


## About the actual launch file

As in fastening_tools.launch, set parameter.

'num_controllers' is the number of U2D2 connections.

'serial_port' is set as many set by 'num_controllers'. 
The value uses the connection name of U2D2.

'conf_dir' sets the path of the config folder.

'fastening_tools' sets the yaml name with gripper information.


## Example of use
### fastener_gripper_control 

Appoint gripper name and rotary speed in 'fastener_gripper_action_client.py'.

Please connect U2D2 and XL320 to PC and run 'demo.launch'.

Two XL320 will be used this time.
ID is 1,2 respectively.

I think that the following operation is possible.

- 1. Turn on the torque of XL-320 and rotate clockwise (wheel mode)
- 2. Stop rotation of the motor with fingers, generate load.
- 3. When the load is detected (rotation speed = 0), the torque of XL-320 is turned off and the rotation stop
- 4. To next motor id .


### screw_control

Please run 'screw_demo.launch'.


