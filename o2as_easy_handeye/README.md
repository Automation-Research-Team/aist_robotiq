# Run handeye calibration in simulator

1: Launch simulator
```bash
$ roslaunch o2as_gazebo o2as_gazebo.launch
```

2: Do following for `$ROBOT_NAME={a_bot, b_bot and c_bot}`

2-1: Launch services for calibration
```bash
$ roslaunch o2as_easy_handeye o2as_calibrate.launch robot_name=$ROBOT_NAME
```

2-2: Run calibration software in a different terminal
```bash
$ rosrun o2as_easy_handeye run_handeye_calibration.py $ROBOT_NAME
```

2-3: Stop the program in 2-1

3. Publish the estimated camera frames, loaded from the calibration files:
```bash
$ roslaunch o2as_easy_handeye o2as_publish_handeye.launch
```
