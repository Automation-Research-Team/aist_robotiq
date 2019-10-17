# Navigation Stack

## 前提

* ロボットの座標フレームの関係がtfを使ってをpublishされている
* 障害物回避のためのセンサ情報が、次のいずれかでpublishされていること
  * `sensor_msgs/LaserScan`
  * `sensor_msgs/PointCloud`
* オドメトリ情報が `nav_msgs/Odometry` の形式で、tfを使ってpublishされていること
* ロボットは `geometry_msgs/Twist` を "cmd_vel"トピックに対してpublishすることで移動すること

環境地図がなくてもNavigation Stackの動作を行うことは可能である。

## マップを使わないNavigation

huskyのROSパッケージ [^husky] にある"husky_navigation"では、環境地図を使わずに自走するための

move_baseに関する設定が含まれている。

これを参考にして、自分のロボットが環境地図なしで目的地へ移動するための設定を確認する。



### First trial

"husky_navigation"からmove_baseに関する設定をコピーして、Fetchの走行に適用できるかを確認する。

使用したファイルは次の通りである。

* launch
  * move_base.launch
  * move_base_mapless.launch
* config
  * costmap_common.yaml
  * costmap_global.yaml
  * costmap_local.yaml
  * planner.yaml

Navigation stackの実行時に依存するパッケージは次の通りである。

* move_base
* navfn
* base_local_planner
* dwa_local_planner

### 実行手順

1. ターミナルを3個立ち上げ、それぞれ次のコマンドを実行する。
   * terminal 1
	```sh
	$ roslaunch fetch_gazebo simulation.launch
	```

	* terminal 2
	```sh
	$ roslaunch yh_navigation move_base_mapless.launch
	```

	* terminal 3
	```sh
	$ rosrun rviz rviz
	```

2. RVizで"robot_state", "map"をそれぞれ追加する。

3. RVizの"2D Nav Goal"ボタンをクリックして、目的地を与える。

### 実行結果2

Fetchが自律移動を行う様子が確認できた。
実行時の`move_base_mapless.launch`のコンソール出力を次に示す。

```sh
 roslaunch aist_navigation move_base_mapless.launch
... logging to /home/parallels/.ros/log/3b4d5d28-b975-11e9-b2db-001c422ee7cc/roslaunch-TSUKUYOMI-11255.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://TSUKUYOMI:43787/

SUMMARY
========

PARAMETERS
 * /move_base/DWAPlannerROS/acc_lim_th: 3.2
 * /move_base/DWAPlannerROS/acc_lim_x: 2.5
 * /move_base/DWAPlannerROS/acc_lim_y: 0
 * /move_base/DWAPlannerROS/latch_xy_goal_tolerance: False
 * /move_base/DWAPlannerROS/max_rot_vel: 1.0
 * /move_base/DWAPlannerROS/max_trans_vel: 0.5
 * /move_base/DWAPlannerROS/max_vel_x: 0.5
 * /move_base/DWAPlannerROS/max_vel_y: 0
 * /move_base/DWAPlannerROS/min_rot_vel: 0.2
 * /move_base/DWAPlannerROS/min_trans_vel: 0.1
 * /move_base/DWAPlannerROS/min_vel_x: 0.0
 * /move_base/DWAPlannerROS/min_vel_y: 0
 * /move_base/DWAPlannerROS/xy_goal_tolerance: 0.2
 * /move_base/DWAPlannerROS/yaw_goal_tolerance: 0.1
 * /move_base/NavfnROS/allow_unknown: True
 * /move_base/NavfnROS/default_tolerance: 0.1
 * /move_base/TrajectoryPlannerROS/acc_lim_theta: 3.2
 * /move_base/TrajectoryPlannerROS/acc_lim_x: 2.5
 * /move_base/TrajectoryPlannerROS/angular_sim_granularity: 0.02
 * /move_base/TrajectoryPlannerROS/controller_frequency: 20.0
 * /move_base/TrajectoryPlannerROS/dwa: True
 * /move_base/TrajectoryPlannerROS/escape_reset_dist: 0.1
 * /move_base/TrajectoryPlannerROS/escape_reset_theta: 0.1
 * /move_base/TrajectoryPlannerROS/escape_vel: -0.1
 * /move_base/TrajectoryPlannerROS/gdist_scale: 1.0
 * /move_base/TrajectoryPlannerROS/heading_lookahead: 0.325
 * /move_base/TrajectoryPlannerROS/heading_scoring: False
 * /move_base/TrajectoryPlannerROS/heading_scoring_timestep: 0.8
 * /move_base/TrajectoryPlannerROS/holonomic_robot: False
 * /move_base/TrajectoryPlannerROS/latch_xy_goal_tolerance: False
 * /move_base/TrajectoryPlannerROS/max_vel_theta: 1.0
 * /move_base/TrajectoryPlannerROS/max_vel_x: 1.0
 * /move_base/TrajectoryPlannerROS/meter_scoring: True
 * /move_base/TrajectoryPlannerROS/min_in_place_vel_theta: 0.2
 * /move_base/TrajectoryPlannerROS/min_vel_theta: -1.0
 * /move_base/TrajectoryPlannerROS/min_vel_x: 0.0
 * /move_base/TrajectoryPlannerROS/occdist_scale: 0.1
 * /move_base/TrajectoryPlannerROS/oscillation_reset_dist: 0.25
 * /move_base/TrajectoryPlannerROS/pdist_scale: 0.75
 * /move_base/TrajectoryPlannerROS/publish_cost_grid_pc: True
 * /move_base/TrajectoryPlannerROS/sim_granularity: 0.02
 * /move_base/TrajectoryPlannerROS/sim_time: 2.0
 * /move_base/TrajectoryPlannerROS/simple_attractor: False
 * /move_base/TrajectoryPlannerROS/vtheta_samples: 20
 * /move_base/TrajectoryPlannerROS/vx_samples: 6
 * /move_base/TrajectoryPlannerROS/xy_goal_tolerance: 0.2
 * /move_base/TrajectoryPlannerROS/yaw_goal_tolerance: 0.1
 * /move_base/base_global_planner: navfn/NavfnROS
 * /move_base/base_local_planner: dwa_local_planner...
 * /move_base/controller_frequency: 5.0
 * /move_base/global_costmap/footprint: [[-0.5, -0.33], [...
 * /move_base/global_costmap/footprint_padding: 0.01
 * /move_base/global_costmap/global_frame: odom
 * /move_base/global_costmap/height: 100.0
 * /move_base/global_costmap/inflation/inflation_radius: 1.0
 * /move_base/global_costmap/obstacle_range: 5.5
 * /move_base/global_costmap/obstacles_laser/laser/clearing: True
 * /move_base/global_costmap/obstacles_laser/laser/data_type: LaserScan
 * /move_base/global_costmap/obstacles_laser/laser/inf_is_valid: True
 * /move_base/global_costmap/obstacles_laser/laser/marking: True
 * /move_base/global_costmap/obstacles_laser/laser/topic: scan
 * /move_base/global_costmap/obstacles_laser/observation_sources: laser
 * /move_base/global_costmap/plugins: [{'type': 'costma...
 * /move_base/global_costmap/publish_frequency: 3.0
 * /move_base/global_costmap/raytrace_range: 6.0
 * /move_base/global_costmap/resolution: 0.05
 * /move_base/global_costmap/robot_base_frame: base_link
 * /move_base/global_costmap/rolling_window: True
 * /move_base/global_costmap/static/map_topic: /map
 * /move_base/global_costmap/static/subscribe_to_updates: True
 * /move_base/global_costmap/track_unknown_space: True
 * /move_base/global_costmap/transform_tolerance: 0.5
 * /move_base/global_costmap/update_frequency: 4.0
 * /move_base/global_costmap/width: 100.0
 * /move_base/local_costmap/footprint: [[-0.5, -0.33], [...
 * /move_base/local_costmap/footprint_padding: 0.01
 * /move_base/local_costmap/global_frame: odom
 * /move_base/local_costmap/height: 10.0
 * /move_base/local_costmap/inflation/inflation_radius: 1.0
 * /move_base/local_costmap/obstacle_range: 5.5
 * /move_base/local_costmap/obstacles_laser/laser/clearing: True
 * /move_base/local_costmap/obstacles_laser/laser/data_type: LaserScan
 * /move_base/local_costmap/obstacles_laser/laser/inf_is_valid: True
 * /move_base/local_costmap/obstacles_laser/laser/marking: True
 * /move_base/local_costmap/obstacles_laser/laser/topic: scan
 * /move_base/local_costmap/obstacles_laser/observation_sources: laser
 * /move_base/local_costmap/plugins: [{'type': 'costma...
 * /move_base/local_costmap/publish_frequency: 3.0
 * /move_base/local_costmap/raytrace_range: 6.0
 * /move_base/local_costmap/resolution: 0.05
 * /move_base/local_costmap/robot_base_frame: base_link
 * /move_base/local_costmap/rolling_window: True
 * /move_base/local_costmap/static/map_topic: /map
 * /move_base/local_costmap/static/subscribe_to_updates: True
 * /move_base/local_costmap/transform_tolerance: 0.5
 * /move_base/local_costmap/update_frequency: 4.0
 * /move_base/local_costmap/width: 10.0
 * /move_base/recovery_behaviour_enabled: True
 * /rosdistro: melodic
 * /rosversion: 1.14.3

NODES
  /
    move_base (move_base/move_base)

ROS_MASTER_URI=http://localhost:11311

process[move_base-1]: started with pid [11319]
[ INFO] [1565224926.995014081, 15.328000000]: Using plugin "obstacles_laser"
[ INFO] [1565224927.007453701, 15.336000000]:     Subscribed to Topics: laser
[ INFO] [1565224927.069284241, 15.356000000]: Using plugin "inflation"
[ INFO] [1565224927.309123028, 15.437000000]: Using plugin "obstacles_laser"
[ INFO] [1565224927.315685276, 15.438000000]:     Subscribed to Topics: laser
[ INFO] [1565224927.437440745, 15.482000000]: Using plugin "inflation"
[ INFO] [1565224927.746227809, 15.631000000]: Created local_planner dwa_local_planner/DWAPlannerROS
[ INFO] [1565224927.765751319, 15.643000000]: Sim period is set to 0.20
[ WARN] [1565224927.785175265, 15.649000000]: Parameter max_trans_vel is deprecated (and will not load properly). Use max_vel_trans instead.
[ WARN] [1565224927.788587705, 15.650000000]: Parameter min_trans_vel is deprecated (and will not load properly). Use min_vel_trans instead.
[ WARN] [1565224927.790045240, 15.650000000]: Parameter max_rot_vel is deprecated (and will not load properly). Use max_vel_theta instead.
[ WARN] [1565224927.794328773, 15.651000000]: Parameter min_rot_vel is deprecated (and will not load properly). Use min_vel_theta instead.
[ INFO] [1565224928.235995388, 15.858000000]: Recovery behavior will clear layer obstacles
[ INFO] [1565224928.252497388, 15.868000000]: Recovery behavior will clear layer obstacles
[ INFO] [1565224928.484156782, 15.967000000]: odom received!
[ INFO] [1565225278.395125092, 127.555000000]: Got new plan
[ INFO] [1565225313.059280628, 138.955000000]: Goal reached
```

この時、global costmapが自動で拡大されていることが確認された。
global costmapの設定とそれによって自動でglobal costmapが生成されることとの関連を調査する必要がある。

---

[^husky]: https://github.com/husky/husky
