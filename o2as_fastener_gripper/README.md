# Introduction
Control the XL320 with ROS.
It is implemented by Action Server.

Install the Dynamixel SDK to control XL320.
The following is the part described in Dockerfile.
```bash
RUN apt-get update && apt-get install -y --no-install-recommends \
	ros-kinetic-dynamixel-sdk \
```
This package controls XL320, so dynamixel-workbench and dynamixel-workbench-msgs are dependent packages.


# How to use
Please connect U2D2 and XL320 to PC.
For connection setting of U2D2, please refer [here](https://gitlab.com/o2as/ur-o2as/blob/develop/udev_rules.md).

Please set the name of U2D2 to o2as_fastener_gripper/launch/fastener_gripper_action.launch.
The following is an example(o2as_fastener_gripper/launch/fastener_gripper_action.launch).
```bash
<param name="connect_name" type="str" value="/dev/for_docker/fastener_gripper_1"/>
<param name="baundrate"    type="int" value="1000000"/>
```

Three XL320 will be used this time.
ID is 1,2,3 respectively.
BaundRate is 1000000.

After connecting and configuring, execute fastener_gripper_test_action.launch.
I think that the following operation is possible.
1.Turn on the torque of 'XL-320' and rotate clockwise (wheel mode)
2.Stop rotation of the motor with fingers, generate load (assuming screws close)
3.When the load is detected (rotation speed = 0), the torque of 'XL-320' is turned off and the rotation stop
4.Next ID(XL320).


# Known issues

to be filled. 
