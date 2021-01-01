<<<<<<< HEAD
artrosのインストール法と使用法（暫定）
====

artros: アーム，カメラ等を制御するROSプラットフォーム

## インストール手順
### ROS環境の準備
最初に，Ubuntu-18.04上に`ros-melodic-desktop-full`をインストールする．次に，python関係のツールをインストールする．
```bash
$ sudo apt-get install python-katkin-tools python-rosdep python-dev python-numpy python-pip python-setuptools python-scipy
```
そして，rosdepを初期化する．
```bash
$ sudo rosdep init
$ rosdep update
```

### artros本体のセットアップ
最初に，`artros`を入手する．
```bash
$ cd catkin_ws/src
$ git clone https://gitlab.com/art-aist-private/artros
```
次に，`artros`が依存しているサブモジュールを取り込む．
```bash
$ cd artros
$ git submodule update --init
```
さらに，依存パッケージをインストールする．
```bash
$ rosdep install -i --from-paths .
```

### Photoneoのカメラドライバをビルドするための準備(必須)
[aist_phoxi_camera/README.md](aist_phoxi_camera/README.md)の`Installation`に従って，Photoneoのカメラドライバパッケージ`aist_phoxi_camera`をビルドする準備をする．

### Photoneo LocalizationSDKを使用した3D CAD matchingパッケージをビルドするための準備(必須)
[aist_localization/README.md](aist_localization/README.md)の`Installation`に従って，Photoneoのカメラドライバパッケージ`aist_localization`をビルドする準備をする．

###	全パッケージのビルド
ワークスペース全体をビルドする．
```bash
$ cd catkin_ws
$ catkin build
```
エラーが出た場合は，必要パッケージがインストールされていないので，適宜手動でインストールする．必要ないと思われるパッケージは，そのディレクトリ内に CATKIN_IGNORE という名前の空ファイルを作れば，コンパイル対象から外すことができる．

##	UR5の使用法
### 準備
ロボットコントローラのIP addressが10.66.171.52でない場合は，artros/aist_bringup/launch/aist_bringup.launch内の変数driver_argsの該当部分を正しいものに書き換える．

### ロボットの起動
以下の手順でロボットを立ち上げる． 
```bash
$ roslaunch aist_bringup aist_bringup.launch [sim:=true]
```
sim:=true を指定すればgazeboが起動してシミュレーションでアームの動作とカメラ画像を確認できる．指定しなければ実ロボットおよび実カメラと接続される． Rvizも一緒に起動するので，Interactive Markerで対話的にアームを動かすことができる． アームには予め3つの姿勢(home, mirrored_home, back)が登録されており，Planningタブから選択することによってその姿勢を取らせることができる．

### 対話的なロボットの操作
コマンドラインからアームを対話的に制御することも可能である．
```bash
$ roslaunch aist_routines aist_interactive.launch config:=aist robot_name:=a_bot camera_name:=a_bot_inside_camera
```
主なコマンドは次のとおりである．
-	**h**:	ホームポジションへ移動
-	**b**:	バックポジションへ移動
-	**o**:	基準姿勢へ移動
-	**q**:	ホームポジションへ移動して終了
-	**X|Y|Z|R|P|W**:	軸(x, y, z, roll, pitch, yaw)選択
-	**数値**:	選択中の軸を指定した数値の位置に動かす
-	**grasp**:	グリッパを閉じる
-	**release**:	グリッパを開く

他にどのようなコマンドがあるかは，直接ソース(artros/aist_routines/scripts/interacitve.py)を参照されたい．

## ハンドアイキャリブレーション
概要を以下に説明するが，詳細は[aist_handeye_calibration/README.md](aist_handeye_calibration/README.md)を参照されたい．
### キャリブレーションの実行
まず，ロボット，カメラ，マーカ検出器およびキャリブレーションサーバを立ち上げる． 
```bash
$ roslaunch aist_handeye_calibration ur10e_handeye_calibration.launch [sim:=true]
```
sim:=true を指定すればgazeboが起動してシミュレータ上で，指定しなければ実ロボットおよび実カメラ上でキャリブレーションが実行される． 
次に，キャリブレーションクライアントを立ち上げる． 
```bash
$ roslaunch aist_handeye_calibration run_calibration.launch config:=ur10e camera_name:=a_bot_camera
```
立ち上がった後，RETキーを押せばロボットが予め定義しておいたいくつかのキーポーズに移動してマーカ画像を取得し，最後にホームポジションに戻ってロボットに対するカメラの位置と姿勢が計算される．計算結果は，`~/.ros/aist_handeye_calibration/a_bot_camera.yaml`に格納される．

###	キャリブレーション結果の検証
まず，ロボット，カメラおよびマーカ検出器を立ち上げる．
```bash
$ roslaunch aist_handeye_calibration ur10e_handeye_calibration.launch check:=true
```
次に，チェッカクライアントを立ち上げる．
```bash
$ roslaunch aist_handeye_calibration check_calibration.launch config:=ur10e camera_name:=a_bot_camera
```
主なコマンドは次のとおりである．
-	**RETURN**:	マーカを撮影し，その中心位置にエフェクタの先端を移動
-	**i**:	初期ポジションへ移動
-	**h**:	ホームポジションへ移動
-	**q**:	ホームポジションへ移動して終了

立ち上がった後，iコマンドで初期ポジション（ワークスペース中央の上方）に移動し，キャリブレーションに使用したマーカをワークスペース上に置いてRETキーを押せば，カメラがマーカを撮影してロボットがその中心にエフェクタ先端を移動する．以後，視野内でマーカを動かしてRETキーを押せば，再びその位置にロボットが移動する．

###	キャリブレーション結果の利用
キャリブレーションパブリッシャを起動して推定されたカメラの位置と姿勢をTFにpublishする．
```bash
$ roslaunch aist_handeye_calibration publish_calibration.launch config:=aist camera_name:=a_bot__inside_camera 
```
これにより，カメラ座標系が`calibrated_a_bot_inside_camera`という名前でTFツリーに挿入される．すなわち，カメラが取得する3次元情報はこの座標系で記述されており，その値はTFによってツリー中の任意の座標系に変換することができる．
=======
# ART ROS packages
ROS wrappers for various projects within ART lab

<br />

## What's so special about this branch?
* nothing, just little bit of instructions and a guide for others<br />

* added support for handeye calibaration using realsenseD435 rgbd camera with aruco marker ID=32, size=0.16, margin=0.01

* calibration accuracy achieved within `0.007m` in position and `0.69` in degrees

* main usage of this branch was to utilize realsenseD435 for pose detection and Pick-n-Place experiment with `UR5` using `MoveIt`

* tested on Ubuntu 18.04, ROS Melodic, RTX 2080-Ti, CUDA 10.1, Python2.7/3.7


<br />

# Usage
## 1. Installation, (I know its messy!)
* this will install everything, `UR5`, `MoveIt`, `realsense`, `aruco` packages
* cd ~/catkin_ws/src
    * git clone https://gitlab.com/art-aist-private/aist_aruco_ros.git
    * git clone https://gitlab.com/art-aist-private/artros.git
        * cd artros/
        * git checkout realsenseD435
        * git submodule update --init
        * rosdep install -i --from-paths .
        * cd ~/catkin_ws/src/artros/aist_phoxi_camera/install-scripts/
            * sudo ./install-phoxi-control.sh
        * cd ~/catkin_ws/src/artros/aist_localization/install-scripts/
            * sudo ./install-photoneo-localization.sh
        * sudo reboot (*restart your pc*)

        * copy these lines to your ~/.bashrc
            * *you don't need them but they are part of other pkgs*
            ```
            ###
            ### PhoXi settings
            ###
            if [ -d /opt/PhotoneoPhoXiControl-1.2.14 ]; then
            export PHOXI_CONTROL_PATH=/opt/PhotoneoPhoXiControl-1.2.14
            export PATH=${PATH}:${PHOXI_CONTROL_PATH}/bin
            export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PHOXI_CONTROL_PATH}/API/lib
            export CPATH=${CPATH}:${PHOXI_CONTROL_PATH}/API/include
            fi

            ###
            ### PhoLocalization settings
            ###
            if [ -d /opt/PhotoneoSDK/Localization ]; then
            export PHO_LOCALIZATION_PATH=/opt/PhotoneoSDK/Localization
            export PATH=${PATH}:${PHO_LOCALIZATION_PATH}/bin
            export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${PHO_LOCALIZATION_PATH}/lib
            export CPATH=${CPATH}:${PHO_LOCALIZATION_PATH}/include
            fi
            ```
        * ***finally***, you can complie
        * cd ~/catkin_ws/
            * catkin_make
            * catkin_make install
            * cd devel
            * source ./setup.bash

        ** `Note:` if you get catkin_make `error` in *robotiq-cri* due to *`byte to str conversion`* do the following
        * go to line https://gitlab.com/art-aist/robotiq-cri/-/blob/devel-aist/robotiq_control/src/robotiq_control/cmodel_urscript.py#L125
        * in method ``buildCommandProgram`` replace line
        ```
        complete_program += program_line
        ```
        with
        ```
        encoding = 'utf-8'
        complete_program += program_line.decode(encoding)
        ```

<br />

<br />

## 2. if installation is successful, test it!
 * In simulation environment
    * roslaunch aist_bringup mocap_bringup.launch sim:=true
    * ![Alt text](images/scsim.png?raw=true "sim environment")
 * In real environment with UR5
    * roslaunch aist_bringup mocap_bringup.launch
    * ![Alt text](images/real.png?raw=true "real robot environment")


<br />
<br />


## 3. To perform handeye calibration using realsenseD435
* go to https://gitlab.com/art-aist-private/artros/-/tree/realsenseD435/aist_handeye_calibration and follow the instructions. or....(*follow below instructions*)
    * use config:=`mocap`
    * use camera_name:=`realsenseD435`

    ** **use this when calibrating realsenseD435**
    * run handeye calibration
    * roslaunch aist_handeye_calibration mocap_handeye_calibration.launch camera_name:=realsenseD435 scene:=mocap_calibration

    * start robot-control
        * roslaunch aist_handeye_calibration run_calibration.launch config:=mocap camera_name:=realsenseD435

    ** **use this when verifying calibration of realsenseD435**
    * to verify handeye calibration
        * roslaunch aist_handeye_calibration mocap_handeye_calibration.launch camera_name:=realsenseD435 check:=true

    * move UR5 ef to calibrated pose of aruco marker
        * roslaunch aist_handeye_calibration check_calibration.launch config:=mocap camera_name:=realsenseD435

<br />
<br />

## 4. If you wish to test further and perform object picking
* clone `yolo6d_ros` pkg https://github.com/avasalya/Yolo6D_ROS.git (***wt missing***? try password: `telexistence`)
* follow instructions and setup as per requirement
* by default, aruco marker pose is published as geometry_msgs/`PoseStamped`
* but Yolo6D ros-wrapper publishes pose as geometry_msgs/`PoseArray`
* so to adapt, change line https://gitlab.com/art-aist-private/artros/-/blob/realsenseD435/aist_routines/src/aist_routines/base.py#L379
```
    est_pose = target_pose.poses[n] #poseArray
    # est_pose = target_pose.pose #poseStamped
```
* `n` is the count of detected object, in my case I detect only 1 onigiri so, n is 0


<br />
<br />


## 5. Try out pick-n-place experiment
* `roslaunch aist_handeye_calibration mocap_handeye_calibration.launch camera_name:=realsenseD435 check:=true`
* activate `yolo6d` conda environment
    * `rosrun yolo6d_ros yolo6d_ros.py pnp`
* `roslaunch yolo6d_ros grasping.launch`
    * afterwards, follow instructions as displayed on the terminal.

<br />

* you should see output similar to this ![Alt text](images/onigiripick.png?raw=true "yolo6d pose")
>>>>>>> origin/realsenseD435

