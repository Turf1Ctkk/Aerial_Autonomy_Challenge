## 文件结构

* `shfiles`: 脚本文件目录
  * `checkcpu.sh`: 查看cpu频率信息
  * `land.sh`: 发布自动降落指令
  * `mapping.sh`: 启动建图相关流程
  * `record.sh`: 开始rosbag包的录制
  * `rspx4.sh`: 连接飞控，雷达，启动fast-lio SLAM
  * `sys.sh`: 将cpu设置为性能模式
  * `takeoff.sh`: 发布自动起飞指令

* `src`: 功能包目录
  * `auto_mission`: 自动任务相关节点
  * `diff_planner`: 
    * `drone_detect`: 集群时检测其他drone
    * `path_searching`: 路径搜索
    * `plan_env`: 地图环境构建
    * `plan_manage`: 规划管理主入口
    * `swarm_bridge`: 集群通信桥接
    * `traj_opt`: 轨迹优化
    * `traj_utils`: 轨迹工具与消息定义
  * `LiDAR_IMU_Init`: LiDAR-IMU联合初始化模块
    * `config`: 配置文件
    * `launch`: 启动文件
    * `msg`: 自定义消息
    * `python_code`: Python工具脚本
    * `src`: 核心源码
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
    * `FAST_LIO`: 状态估计
    * `px4ctrl`: 控制器
      * `src`
        * `controller.h/cpp`: 控制器计算部分
        * `input.h/cpp`: 输入数据处理
        * `px4ctrl_node`: px4ctrl节点主入口
        * **`PX4CtrlFSM.h/cpp`: 状态机**
        * `PX4CtrlParam.h/cpp`: 参数处理
        * **`New`**: 
          * `On-manifold MPC`
  * `uav_simulator`: 仿真
  * `user_command`: 用户任务命令模块
    * `multipoint`: 多点任务执行
  * `utils`: 通用类