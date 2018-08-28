# Introduction
Control the XL320 with ROS.

It is implemented by Action Server.

Turn in the 'CCW' direction.

Hold a motor in check with a finger, and the turn stops if resistance becomes big.

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
For connection setting of U2D2, please refer [here](https://gitlab.com/o2as/ur-o2as/blob/develop/udev_rules.md).

I use U2D2 five with this package.
Set param in 'demo.launch'.
Please set the maximum number of connections and connection destination.

* ID of XL-320
Must set motor id to move.

It is a unique number to identify Dynamixel.

Please excute 'motor_id_set.launch'.

The information of a connected motor is displayed.
Because an input screen of the ID setting starts, please set new ID.

Please be cautious not to have the same IDs for the connected dynamixels. You may face communication issues or may not be able to search when IDs overlap.

Specifically, please refer to [here](http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm#Actuator_Address_03).

* config/gripper.yaml
Please set a name for the set motor ID.
Item names are 'name' and 'motor_id'.
You will set parameters with that name.

# How to use
Appoint gripper name and rotary speed in 'fastener_gripper_action_client.py'.

Please connect U2D2 and XL320 to PC and run 'demo.launch'.

Two XL320 will be used this time.
ID is 1,2 respectively.

I think that the following operation is possible.

* 1. Turn on the torque of XL-320 and rotate clockwise (wheel mode)
* 2. Stop rotation of the motor with fingers, generate load (assuming screws close)
* 3. When the load is detected (rotation speed = 0), the torque of XL-320 is turned off and the rotation stop
* 4. To next motor.


