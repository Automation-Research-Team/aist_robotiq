# Introduction

This package should offer vision services and actions to the state machine and other nodes.

See the o2as_skills package, which should use this package and has a similar structure.

# Available ROS services

```
FindObject
```

# How to use

Run object detection demo. 

    ```
    roslaunch o2as_vision demo.launch
    ```

This demonstration doing,
- launch camera node (realsense sr300)
- save point cloud and iamge file captured by the camera node
- search object in the image and add detected object into planning_scene of moveit.

If you have no trained model, you need to train model in advance.

    ```
    roslaunch o2as_cad_matching train_assembly_realsense_************.launch
    ```
