## 机载电脑NUC11TNKv5

系统：Ubuntu20.04

ros-noetic-full

### MAVROS-PX4环境

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

### RealSense SDK & realsense-ros

版本统一：RealSense SDK: v2.48.0（新的版本不再支持T265）; realsense-ros: build 2.3.1

安装时相机不要连在设备上

#### **RealSense SDK安装**

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

#### **realsense-ros安装**

```bash
git clone -b 2.3.1 https://github.com/IntelRealSense/realsense-ros
cd realsense-ros
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install

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

### [QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html)

### LD14P

数据格式参见开发手册，`LD14P-Python3.0.py` 为读取数据demo文件

#### Windows

1. 安装CP2102驱动
2. 设备管理器查看端口号，在 `LD14P-Python3.0.py` 中修改为对应端口号，运行py文件即可读取
3. （可选）安装LdsPointCloudViewer进行扫描结果可视化

#### Ubuntu

1. 将雷达与设备连接
2. 执行 `wheeltec_udev.sh  ` 脚本将ttyUSB0端口名修改为wheeltec_lidar，重新插拔雷达生效（不影响其他USB外部设备的端口号）。执行 `LD14P-Python3.0.py` 文件读取数据
3. 将数据通过ROS的/scan话题发出：

```bash
#创建工作空间，复制功能包
catkin_make
source ./devel/setup.bash
roslaunch ldlidar ld14p.launch
#rviz可视化时，fixed_frame手动改为/laser，添加laserscan，话题为/scan
```



MAVLink：轻量级的通信协议，一些无人机硬件平台如Pixhawk、PX4、ArduPilot等就是使用MAVLink通讯，包含了许多无人机相关的信息和命令，例如无人机的状态、传感器数据、电池电量等。

MAVROS：用于将ROS和MAVLink协议连接起来，以实现ROS与无人机之间的通信和控制。
