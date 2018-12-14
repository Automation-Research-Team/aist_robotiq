# Introduction

This package provides sensor data from the Xela robotics tactile sensor with 16 taxels.
The sensor node publishes the XelaSensorStamped message to 
The sensor data is in the "data" member of the message as a float array [x1,y1,z1,x2,y2,z2,...,x16,y16,z16]

The "calibrate" action is available to re-zero the sensor.

# Initial Setup
* Install requirements

    ```bash
    pip install opencv-python
    pip install python-can
    ```

# How to use

1. Connect xela_sensor to PC.
2. Before running, do:

    ```bash
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set up can0
    ```

3. Run demonstration to visualize the states of taxels as circles.

    ```
    roslaunch o2as_xela_sensor demo.launch
    ```
