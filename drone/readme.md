## 文件结构

* `shfiles`: 脚本文件目录
  * `checkcpu.sh`: 查看cpu频率信息
  * `land.sh`: 发布自动降落指令
  * `make.sh`: 编译工作空间
  * `record.sh`: 开始rosbag包的录制
  * `rspx4.sh`: 连接飞控，雷达，启动fast-lio SLAM
  * `sys.sh`: 将cpu设置为性能模式
  * `takeoff.sh`: 发布自动起飞指令

* `src`: 功能包目录
  * `livox_ros_driver2`: 雷达与ROS通信驱动包
  * `planner`:  ego-swarm-planner
    * `bspline_opt`: B样条曲线优化器
    * `drone_detect`: 检测其他飞行器
    * `path_searching`: 动态A*搜索
    * `plan_env`: 建立栅格地图
    * `plan_manage`: planner主入口
    * `rosmsg_tcp_bridge`: 集群通信桥梁
    * `traj_utils`: 轨迹实用类
  * `realflight`
    * `FAST_LIO`: SLAM算法
    * `px4ctrl`: 控制器
      * `src`
        * `controller.h/cpp`: 控制器计算部分
        * `input.h/cpp`: 输入数据处理
        * `px4ctrl_node`: px4ctrl节点主入口
        * **`PX4CtrlFSM.h/cpp`: 状态机**
        * `PX4CtrlParam.h/cpp`: 参数处理
  * `uav_simulator`: ego-planner仿真部分
  * `utils`: 通用类