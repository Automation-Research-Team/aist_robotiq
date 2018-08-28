1: Launch simulator
```bash
$ roslaunch o2as_gazebo o2as_gazebo.launch
```

2: Launch calibration software
```bash
$ roslaunch o2as_easy_handeye o2as_calibrate.launch 
```

3: In Rviz GUI, `File > Open config` and select 
`o2as_moveit_config/launch/o2as_easy_handeye`.

4: Move `b_bot` with GUI, where the AR marker is visible and its pose is 
detected. See the left bottom panel of the camera image.

5: Take a sample for calibration:
```bash
$ rosservice call /o2as_easy_handeye_b_bot_eye_on_base/take_sample
```

6: Repeat 5 several times. NOTE: I see that taking many samples is not always 
good. I guess each sample should not be far from each other for numerical 
stability.

7: Calculate calibration parameters and save them to a file:
```bash
$ rosservice call /o2as_easy_handeye_b_bot_eye_on_base/compute_calibration
$ rosservice call /o2as_easy_handeye_b_bot_eye_on_base/save_calibration
```

8: Publish the estimated camera frame, loaded from the calibration file:
```bash
$ roslaunch o2as_easy_handeye o2as_publish.launch
```
