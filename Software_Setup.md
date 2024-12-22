## 机载电脑NUC11TNKv5

系统：Ubuntu20.04

ros-noetic-full

## MAVROS-PX4环境

包括但不限于以下依赖项：

```bash
sudo apt install -y \
ninja-build \
exiftool \
python-argparse \
python-empy \
python-toml \
python-numpy \
python-yaml \
python-dev \
python-pip \
ninja-build \
protobuf-compiler \
libeigen3-dev \
genromfs

pip install \
pandas \
jinja2 \
pyserial \
cerberus \
pyulog \
numpy \
toml \
pyquaternion
```

#### **MAVROS安装**：

MAVLink：轻量级的通信协议，一些无人机硬件平台如Pixhawk、PX4、ArduPilot等就是使用MAVLink通讯，包含了许多无人机相关的信息和命令，例如无人机的状态、传感器数据、电池电量等。

MAVROS：用于将ROS和MAVLink协议连接起来，以实现ROS与无人机之间的通信和控制。

```bash
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
```

**Clone PX4源码（目前使用Github上的最新版本，待确定统一的版本。传言有固件版本存在bug，且飞行效果有差别）**

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
git submodule update --init --recursive
sudo bash ./Tools/setup/ubuntu.sh
sudo bash ./Tools/setup/ubuntu.sh --fix-missing

make px4_sitl_default gazebo
```

**添加环境变量**

```bash
sudo gedit ~/.bashrc
#加入：
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```

**测试mavros和px4是否可以连接，以及在Gazebo中显示：**

```bash
roslaunch px4 mavros_posix_sitl.launch
```

**查看话题是否正常：**

```bash
rostopic list
rostopic echo /mavros/state
```

#### Gazebo仿真demo

**启动Mavros, Gazebo仿真环境**

```bash
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
roslaunch px4 posix_sitl.launch
```

**或单个命令启动**

```bash
roslaunch px4 mavros_posix_sitl.launch
```

**启动外部控制节点**

```bash
mkdir -p mavros_px4_demo/src
cd mavros_px4_demo/src
catkin_create_pkg offboard roscpp std_msgs geometry_msgs mavros_msgs
```

创建offb_node.cpp文件在包的src下，建立链接（代码地址[MAVROS Offboard control example (C++) | PX4 User Guide](https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html)）

```bash
rosrun offboard offb_node
```

```
rostopic echo /mavros/state
```

无人机将升至2m左右，相应的状态变成offboard模式

## [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)



## Mid-360 & Fast-lio

### 1. 修改本机静态ip地址

```bash
ifconfig
# 查看本机的网口名称，假设为enxxx
sudo ifconfig enxxx 192.168.1.50
```

设置-网络：开启/选用被设置成静态ip地址的网口

### 2. Livox SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install

# 快速测试 需要将mid360_config.json中的本机ip改为192.168.1.50
cd samples/livox_lidar_quick_start
./livox_lidar_quick_start ../../../samples/livox_lidar_quick_start/mid360_config.json
# 无报错且一直在打印point cloud handle和Imu data callback即为正常
```

删除Livox SDK2

```bash
sudo rm -r Livox-SDK2
sudo rm -rf /usr/local/lib/liblivox_lidar_sdk_*
sudo rm -rf /usr/local/include/livox_lidar_*
```

### 3. Livox ROS + Fast-lio

```bash
mkdir -p fast-lio/src
cd fast-lio/src
# 工作空间将包含两个功能包：livox_ros_driver2, FAST_LIO. FAST_LIO的编译依赖livox_ros_driver2，需要先编译livox_ros_driver2

# 1. livox_ros_driver2
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh ROS1

# 修改Mid-360的IP
# 获取IP方式：（一般第一种方法即可，对应出厂默认IP地址；如果被修改过，通过第二种方法可获得IP地址）
# 1. 查看SN码后两位XX，IP则为192.168.1.1XX
# 2. 通过wireshark读取正在发数据的IP地址
	ifconfig
	# 假设网口名称为enxxx
	sudo apt-get install wireshark
	sudo wireshark
	# 点击enxxx，Source代表雷达IP地址，Destination代表本机接收网口（地址应为手动设置的192.168.1.50）
# 修改fast-lio/src/livox_ros_driver2/config/MID360_config.json
# 将host_net_info中的4个IP都改为192.168.1.50，将lidar_configs中的ip字段改为查询到的Mid-360的IP地址

# 运行测试
cd fast-lio
source devel/setup.bash
roslaunch livox_ros_driver2 rviz_MID360.launch
# 正常来说会看到rviz中显示出点云

# 查看发布的话题，包含/livox/lidar, /livox/imu
rostopic list

# 2. FAST_LIO
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init

# 修改FAST_LIO源码中有关livox_ros_driver的内容，改为livox_ros_driver2
# 需修改的文件：CMakelists.txt, package.xml, src/preprocess.h, /src/laserMapping.cpp, /src/preprocess.cpp

cd fast-lio
catkin_make
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
# 另一个终端
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch
```

### 4. Faster-lio

```bash
mkdir -p faster-lio/src
cd faster-lio/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh ROS1

cd faster-lio/src
git clone https://github.com/gaoxiang12/faster-lio.git
# 删除thirdparty下的livox_ros_driver，注释掉CMakeLists中的有关依赖，将其他文件中的livox_ros_driver改为livox_ros_driver2

cd faster-lio
catkin_make
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
# 另一个终端
source devel/setup.bash
roslaunch faster_lio mapping_mid360.launch
```

在config文件夹下加入文件`mid360.yaml`

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/livox/imu"
  time_sync_en: false         # ONLY turn on when external time synchronization is really not possible
 
preprocess:
  lidar_type: 1                # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR,
  scan_line: 4
  blind: 0.5
  time_scale: 1e-3
 
mapping:
  acc_cov: 0.1
  gyr_cov: 0.1
  b_acc_cov: 0.0001
  b_gyr_cov: 0.0001
  fov_degree: 360
  det_range: 100.0
  extrinsic_est_en: false      # true: enable the online estimation of IMU-LiDAR extrinsic
  extrinsic_T: [ -0.011, -0.02329, 0.04412  ]
  extrinsic_R: [ 1, 0, 0,
                 0, 1, 0,
                 0, 0, 1 ]
 
publish:
  path_publish_en: false
  scan_publish_en: true       # false: close all the point cloud output
  scan_effect_pub_en: true    # true: publish the pointscloud of effect point
  dense_publish_en: false       # false: low down the points number in a global-frame point clouds scan.
  scan_bodyframe_pub_en: true  # true: output the point cloud scans in IMU-body-frame
 
path_save_en: true                 # 保存轨迹，用于精度计算和比较
 
pcd_save:
  pcd_save_en: true
  interval: -1                 # how many LiDAR frames saved in each pcd file;
  # -1 : all frames will be saved in ONE pcd file, may lead to memory crash when having too much frames.
 
feature_extract_enable: false
point_filter_num: 3
max_iteration: 3
filter_size_surf: 0.5
filter_size_map: 0.5             # 暂时未用到，代码中为0， 即倾向于将降采样后的scan中的所有点加入map
cube_side_length: 1000
 
ivox_grid_resolution: 0.5        # default=0.2
ivox_nearby_type: 18             # 6, 18, 26
esti_plane_threshold: 0.1        # default=0.1
```

在launch文件夹下加入文件`mapping_mid360.launch`

```yaml
<launch>
<!-- Launch file for Livox AVIA LiDAR -->
 
	<arg name="rviz" default="true" />
 
	<rosparam command="load" file="$(find faster_lio)/config/mid360.yaml" />
 
	<param name="feature_extract_enable" type="bool" value="0"/>
	<param name="point_filter_num_" type="int" value="3"/>
	<param name="max_iteration" type="int" value="3" />
	<param name="filter_size_surf" type="double" value="0.5" />
	<param name="filter_size_map" type="double" value="0.5" />
	<param name="cube_side_length" type="double" value="1000" />
	<param name="runtime_pos_log_enable" type="bool" value="1" />
    <node pkg="faster_lio" type="run_mapping_online" name="laserMapping" output="screen" />
 
	<group if="$(arg rviz)">
	<node launch-prefix="nice" pkg="rviz" type="rviz" name="rviz" args="-d $(find faster_lio)/rviz_cfg/loam_livox.rviz" />
	</group>
 
</launch>
```



## RealSense SDK & realsense-ros

版本统一：RealSense SDK: v2.48.0（新的版本不再支持T265）; realsense-ros: build 2.3.1

安装时相机不要连在设备上

### **RealSense SDK安装**

```bash
sudo apt-get install libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libusb-1.0-0-dev pkg-config
sudo apt-get install libglfw3-dev
sudo apt-get install libssl-dev
sudo apt-get install ros-noetic-ddynamic-reconfigure

git clone -b v2.48.0 https://github.com/IntelRealSense/librealsense
cd librealsense

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

mkdir build
cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true
sudo make uninstall
make clean && make -j8
sudo make install

#测试
realsense-viewer
```

### **realsense-ros安装**

```bash
mkdir -p realsense-ros/src/
cd realsense-ros/src
git clone -b 2.3.1 https://github.com/IntelRealSense/realsense-ros
cd realsense-ros
catkin_make

sudo gedit ~/.bashrc
#加入语句
source ~/realsense-ros/devel/setup.bash

#测试
source ~/.bashrc
#D435
roslaunch realsense2_camera rs_camera.launch
#T265
roslaunch realsense2_camera rs_t265.launch
rostopic list
```
