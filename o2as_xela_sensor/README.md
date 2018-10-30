# Introduction

This package provides sensor data acquisition from the Xela robotics tactile sensor with 16 taxels.
The sensor node publishes XelaSensorStamped message.
The sensor data is saved to the "data" member of the message as float array [x1,y1,z1,x2,y2,z2,...,x16,y16,z16]

# Initial Setup
* Install package

    ```bash
    pip install python-can
    ```

# How to use

1. Connect xela_sensor to PC.
1. Before running it you must do the following.

    ```bash
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set up can0
    ```

1. Run demonstration to visualize the states of taxels as circles.

    ```
    roslaunch o2as_xela_sensor demo.launch
    ```
