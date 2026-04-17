#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

// 添加到全局变量部分
// 夹爪控制相关
int gripper_serial_fd = -1;              // 串口文件描述符
bool use_ball_grab = false;              // 是否启用抓球功能
bool use_ball_drop = false;              // 是否启用放球功能
Eigen::Vector3d ball_to_target_offset(-0.5, 0.0, 1.03);  // 小球相对于目标的偏移
Eigen::Vector3d fixed_grab_offset(0.016, 0.0, 0.05);     // 固定抓取偏移
Eigen::Vector3d drop_to_target_offset(0.5, 0.0, 0.4);    // 投放区相对于目标的偏移

// 全局通信对象
ros::Publisher takeoff_pub;
ros::Publisher waypoint_pub;
ros::Publisher yaw_pub;
ros::Publisher enable_pub;
ros::Publisher disable_pub;
ros::Publisher detection_enable_pub;  // 目标检测启用/禁用
ros::Publisher detection_mode_pub;    // 新增：目标检测模式设置

// 无人机状态
bool initial_position_received = false;
Eigen::Vector3d drone_initial_position(0, 0, 0);

// 目标数据结构
struct DetectedTarget {
    Eigen::Vector3d position;
    std::string color;
};

std::vector<DetectedTarget> first_search_results;  // 第一次搜索的所有目标
std::vector<DetectedTarget> second_search_results; // 第二次搜索的所有目标
std::vector<DetectedTarget> detected_targets;      // 当前搜索点检测到的目标
DetectedTarget first_target;                       // 第一次搜索确认的目标
DetectedTarget second_target;                      // 第二次搜索确认的目标
DetectedTarget final_target;                       // 最终目标

// 里程计数据
Eigen::Vector3d current_position(0, 0, 0);
Eigen::Vector3d current_velocity(0, 0, 0);
bool odom_received = false;

// 航点到达判断参数
double position_threshold = 0.2;     // 位置误差阈值(米)
double velocity_threshold = 0.3;     // 速度阈值(米/秒)
double arrival_duration = 2.0;       // 条件满足持续时间(秒)
double waypoint_timeout = 25.0;      // 航点超时时间(秒)

// 航点结构体
struct Waypoint {
    double x;      // X坐标
    double y;      // Y坐标 
    double z;      // Z坐标(高度)
    double yaw;    // 偏航角(弧度)
    double delay;  // 发布后的延时(秒)
};

// 初始化夹爪串口
bool initGripperSerial(const std::string& port_name) {
    // 打开串口设备
    gripper_serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (gripper_serial_fd < 0) {
        ROS_ERROR("[Mission] Failed to open gripper serial port %s", port_name.c_str());
        return false;
    }

    // 设置串口参数
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    // 获取当前设置
    if (tcgetattr(gripper_serial_fd, &tty) != 0) {
        ROS_ERROR("[Mission] Failed to get serial port attributes");
        close(gripper_serial_fd);
        gripper_serial_fd = -1;
        return false;
    }
    
    // 设置波特率 (115200)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    // 8N1 (8位数据位, 无校验, 1位停止位)
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    
    // 无流控制
    tty.c_cflag &= ~CRTSCTS;
    
    // 原始模式
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    
    // 设置新属性
    if (tcsetattr(gripper_serial_fd, TCSANOW, &tty) != 0) {
        ROS_ERROR("[Mission] Failed to set serial port attributes");
        close(gripper_serial_fd);
        gripper_serial_fd = -1;
        return false;
    }
    
    ROS_INFO("[Mission] Gripper serial port initialized: %s", port_name.c_str());
    return true;
}

// 向夹爪发送控制命令
void sendGripperCommand(bool close_gripper) {
    if (gripper_serial_fd < 0) {
        ROS_WARN("[Mission] Gripper serial port not initialized, cannot send command");
        return;
    }
    
    // 准备要发送的16进制命令
    unsigned char cmd[4];
    if (close_gripper) {
        // 闭合夹爪命令: ff fe bb ee
        cmd[0] = 0xff;
        cmd[1] = 0xfe;
        cmd[2] = 0xbb;
        cmd[3] = 0xee;
        ROS_INFO("[Mission] Sending gripper CLOSE command: ff fe bb ee");
    } else {
        // 张开夹爪命令: ff fe aa ee
        cmd[0] = 0xff;
        cmd[1] = 0xfe;
        cmd[2] = 0xaa;
        cmd[3] = 0xee;
        ROS_INFO("[Mission] Sending gripper OPEN command: ff fe aa ee");
    }
    
    // 发送命令到串口
    int result = write(gripper_serial_fd, cmd, 4);
    
    if (result < 0) {
        ROS_ERROR("[Mission] Failed to send gripper command");
    } else {
        ROS_INFO("[Mission] Gripper command sent successfully (%d bytes)", result);
    }
}

// 延时函数（非阻塞）
void delay(double seconds) {
    ros::Duration(seconds).sleep();
    ros::spinOnce();
}

// 起飞控制
void sendTakeoff() {
    quadrotor_msgs::TakeoffLand msg;
    msg.takeoff_land_cmd = 1;
    takeoff_pub.publish(msg);
    ROS_INFO("[Mission] Takeoff command sent");
}

// 降落控制
void sendLand() {
    quadrotor_msgs::TakeoffLand msg;
    msg.takeoff_land_cmd = 2;
    takeoff_pub.publish(msg);
    ROS_INFO("[Mission] Landing command sent");
}

// 发布航点
void publishWaypoint(const Waypoint& wp) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose.position.x = wp.x;
    pose.pose.position.y = wp.y;
    pose.pose.position.z = wp.z;

    std_msgs::Float32 yaw_msg;
    yaw_msg.data = wp.yaw;

    waypoint_pub.publish(pose);
    yaw_pub.publish(yaw_msg);
    
    ROS_INFO("[Mission] Waypoint published: (%.2f, %.2f, %.2f) Yaw=%.2f", 
        wp.x, wp.y, wp.z, wp.yaw);
}

// 启用规划器控制
void enablePlanner() {
    std_msgs::Empty msg;
    enable_pub.publish(msg);
    ROS_INFO("[Mission] Planner enabled");
}

// 停止规划器控制
void disablePlanner() {
    std_msgs::Empty msg;
    disable_pub.publish(msg);
    ROS_INFO("[Mission] Planner disabled");
}

// 启用/禁用目标检测
void enableTargetDetection(bool enable) {
    std_msgs::Bool msg;
    msg.data = enable;
    detection_enable_pub.publish(msg);
    ROS_INFO("[Mission] Target detection %s", enable ? "enabled" : "disabled");
}

// 设置目标检测模式
void setDetectionMode(bool multi_mode, const std::string& target_color = "all") {
    std_msgs::String msg;
    if (multi_mode) {
        // 多目标模式，可以指定颜色
        msg.data = "multi:" + target_color;
        ROS_INFO("[Mission] Set target detection to multi-target mode, target color: %s", target_color.c_str());
    } else {
        // 单目标模式
        msg.data = "single";
        ROS_INFO("[Mission] Set target detection to single-target mode");
    }
    detection_mode_pub.publish(msg);
    delay(0.1); // 短暂延时确保消息被处理
}

// 里程计回调函数 - 更新无人机当前位置和速度
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 更新位置
    current_position.x() = msg->pose.pose.position.x;
    current_position.y() = msg->pose.pose.position.y;
    current_position.z() = msg->pose.pose.position.z;
    
    // 更新速度
    current_velocity.x() = msg->twist.twist.linear.x;
    current_velocity.y() = msg->twist.twist.linear.y;
    current_velocity.z() = msg->twist.twist.linear.z;
    
    odom_received = true;
}

// 初始位置回调
void initialPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!initial_position_received) {
        drone_initial_position.x() = msg->pose.position.x;
        drone_initial_position.y() = msg->pose.position.y;
        drone_initial_position.z() = msg->pose.position.z;
        
        initial_position_received = true;
        
        ROS_INFO("[Mission] Received drone initial position: (%.2f, %.2f, %.2f)", 
         drone_initial_position.x(), drone_initial_position.y(), drone_initial_position.z());
    }
}

// 目标位置回调 - 更新为从frame_id获取颜色
void targetPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (first_target.color != "" && second_target.color != "") return;  // 如果已经完成两次搜索，不再接收
    
    DetectedTarget target;
    target.position.x() = msg->pose.position.x;
    target.position.y() = msg->pose.position.y;
    target.position.z() = msg->pose.position.z;
    target.color = msg->header.frame_id;  // 从frame_id获取颜色
    
    detected_targets.push_back(target);
    
    ROS_INFO("[Mission] Received target: color=%s, position=(%.2f, %.2f, %.2f), current detection count: %zu", 
         target.color.c_str(), target.position.x(), target.position.y(), target.position.z(), 
         detected_targets.size());
}

// 将场地绝对坐标转换为相对于无人机初始位置的相对坐标
Waypoint transformToRelativeWaypoint(double x_abs, double y_abs, double z_abs, double yaw, double delay) {
    Waypoint wp;
    wp.x = x_abs - drone_initial_position.x();
    wp.y = y_abs - drone_initial_position.y();
    wp.z = z_abs;
    wp.yaw = yaw;
    wp.delay = delay;
    
    ROS_INFO("[Mission] Convert coordinates: absolute(%.2f, %.2f, %.2f) -> relative(%.2f, %.2f, %.2f)", 
         x_abs, y_abs, z_abs, wp.x, wp.y, wp.z);
             
    return wp;
}

// 检查无人机是否到达航点
bool isWaypointReached(const Waypoint& wp) {
    if (!odom_received) {
        return false;
    }
    
    // 计算与目标航点的位置误差
    double dx = current_position.x() - wp.x;
    double dy = current_position.y() - wp.y;
    double dz = current_position.z() - wp.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    
    // 计算速度大小
    double velocity = current_velocity.norm();
    
    // 检查条件是否满足
    bool position_ok = (distance < position_threshold);
    bool velocity_ok = (velocity < velocity_threshold);
    
    if (position_ok && velocity_ok) {
        ROS_INFO_THROTTLE(1.0, "[Mission] Approaching waypoint: distance=%.2f m, velocity=%.2f m/s", distance, velocity);
    }
    
    return position_ok && velocity_ok;
}

// 等待无人机到达航点或超时
bool waitForWaypointReached(const Waypoint& wp, double timeout) {
    ros::Time start_time = ros::Time::now();
    ros::Time arrival_start_time;
    bool arrival_timer_started = false;
    
    ros::Rate rate(10);  // 10Hz的检查频率
    
    ROS_INFO("[Mission] Waiting for waypoint arrival, timeout: %.1f seconds", timeout);
    
    while (ros::ok()) {
        ros::spinOnce();
        
        // 检查是否超时
        double elapsed = (ros::Time::now() - start_time).toSec();
        if (elapsed > timeout) {
            ROS_WARN("[Mission] Waypoint waiting timeout (%.1f seconds)", elapsed);
            return false;
        }
        
        // 检查是否到达航点
        if (isWaypointReached(wp)) {
            if (!arrival_timer_started) {
                arrival_start_time = ros::Time::now();
                arrival_timer_started = true;
                ROS_INFO("[Mission] Waypoint approaching condition met, starting timer (need to maintain for %.1f seconds)", arrival_duration);
            } else {
                // 检查条件持续满足的时间
                double duration = (ros::Time::now() - arrival_start_time).toSec();
                if (duration >= arrival_duration) {
                    ROS_INFO("[Mission] Waypoint reached! Condition maintained for %.1f seconds", duration);
                    return true;
                }
            }
        } else {
            // 如果条件不再满足，重置计时器
            if (arrival_timer_started) {
                ROS_INFO("[Mission] Waypoint approaching condition no longer met, resetting timer");
                arrival_timer_started = false;
            }
        }
        
        rate.sleep();
    }
    
    return false;
}

// 计算检测到的目标平均位置 - 按颜色分组计算
std::map<std::string, DetectedTarget> calculateAverageTargetsByColor() {
    std::map<std::string, std::vector<Eigen::Vector3d>> targets_by_color;
    std::map<std::string, DetectedTarget> average_targets;
    
    if (detected_targets.empty()) {
        return average_targets;
    }
    
    // 按颜色分组
    for (const auto& target : detected_targets) {
        targets_by_color[target.color].push_back(target.position);
    }
    
    // 计算每种颜色的平均位置
    for (const auto& entry : targets_by_color) {
        const std::string& color = entry.first;
        const std::vector<Eigen::Vector3d>& positions = entry.second;
        
        // 只要有至少一个样本就计算平均值，不再要求最少3个样本
        if (!positions.empty()) {
            Eigen::Vector3d avg_pos = Eigen::Vector3d::Zero();
            for (const auto& pos : positions) {
                avg_pos += pos;
            }
            avg_pos /= positions.size();
            
            DetectedTarget avg_target;
            avg_target.position = avg_pos;
            avg_target.color = color;
            average_targets[color] = avg_target;
            
            ROS_INFO("[Mission] Calculate average position for color %s: (%.2f, %.2f, %.2f), sample count: %zu", 
                color.c_str(), avg_pos.x(), avg_pos.y(), avg_pos.z(), positions.size());
        }
    }
    
    return average_targets;
}

// 修改executeFirstTargetSearch函数
bool executeFirstTargetSearch(const std::vector<std::vector<double>>& search_points, double search_height, double detection_time) {
    ROS_INFO("[Mission] Starting first target search, total search points: %zu", search_points.size());
    
    // 设置为单目标模式
    setDetectionMode(false);
    
    for (size_t i = 0; i < search_points.size(); i++) {
        // 清空之前的检测结果
        detected_targets.clear();
        
        // 转换当前搜索点坐标
        Waypoint search_wp = transformToRelativeWaypoint(
            search_points[i][0], 
            search_points[i][1], 
            search_height, 
            0.0, // yaw 
            waypoint_timeout  // 使用超时参数
        );
        
        // 发布搜索点航点
        ROS_INFO("[Mission] Flying to search point %zu/%zu: (%.2f, %.2f, %.2f)", 
        i+1, search_points.size(), search_points[i][0], search_points[i][1], search_height);
        publishWaypoint(search_wp);
        
        // 等待到达搜索点
        waitForWaypointReached(search_wp, search_wp.delay);
        
        // 开启目标检测
        ROS_INFO("[Mission] At search point %zu, enabling single-target detection for %.1f seconds", i+1, detection_time);
        enableTargetDetection(true);
        
        // 等待检测时间
        delay(detection_time);
        
        // 关闭目标检测
        enableTargetDetection(false);
        
        // 计算各颜色目标的平均位置
        auto avg_targets = calculateAverageTargetsByColor();
        
        // 记录所有有效的目标结果
        for (const auto& entry : avg_targets) {
            first_search_results.push_back(entry.second);
        }
        
        // 如果发现了有效目标，选择第一个作为结果并结束搜索
        if (!avg_targets.empty()) {
            first_target = avg_targets.begin()->second;
            ROS_INFO("[Mission] First target search successful: color=%s, position=(%.2f, %.2f, %.2f)",
                first_target.color.c_str(), 
                first_target.position.x(), first_target.position.y(), first_target.position.z());
            return true;
        }
        
        ROS_INFO("[Mission] No valid target detected at search point %zu, continuing search", i+1);
    }
    
    ROS_WARN("[Mission] Completed all search points, but no valid target found");
    return false;
}

// 执行第二次目标搜索 - 按第一次搜索的颜色筛选
bool executeSecondTargetSearch(const std::vector<std::vector<double>>& search_points, double search_height, double detection_time) {
    if (first_target.color.empty()) {
        ROS_ERROR("[Mission] First search did not find a target, cannot execute second search");
        return false;
    }
    
    ROS_INFO("[Mission] Starting second target search, looking for color %s targets, total search points: %zu", 
        first_target.color.c_str(), search_points.size());
    
    // 设置为多目标模式，并指定第一次搜索找到的颜色
    setDetectionMode(true, first_target.color);
    
    for (size_t i = 0; i < search_points.size(); i++) {
        // 清空之前的检测结果
        detected_targets.clear();
        
        // 转换当前搜索点坐标
        Waypoint search_wp = transformToRelativeWaypoint(
            search_points[i][0], 
            search_points[i][1], 
            search_height, 
            0.0, // yaw 
            waypoint_timeout  // 使用超时参数
        );
        
        // 发布搜索点航点
        ROS_INFO("[Mission] Flying to search point %zu/%zu: (%.2f, %.2f, %.2f)", 
            i+1, search_points.size(), search_points[i][0], search_points[i][1], search_height);
        publishWaypoint(search_wp);
        
        // 等待到达搜索点
        waitForWaypointReached(search_wp, search_wp.delay);
        
        // 开启目标检测
        ROS_INFO("[Mission] At search point %zu, enabling multi-target detection for %.1f seconds", i+1, detection_time);
        enableTargetDetection(true);
        
        // 等待检测时间
        delay(detection_time);
        
        // 关闭目标检测
        enableTargetDetection(false);
        
        // 计算各颜色目标的平均位置
        auto avg_targets = calculateAverageTargetsByColor();
        
        // 查找与第一次搜索颜色相同的目标
        auto it = avg_targets.find(first_target.color);
        if (it != avg_targets.end()) {
            second_target = it->second;
            ROS_INFO("[Mission] Second target search successful: color=%s, position=(%.2f, %.2f, %.2f)",
                second_target.color.c_str(), 
                second_target.position.x(), second_target.position.y(), second_target.position.z());
            return true;
        }
        
        ROS_INFO("[Mission] No target of matching color (%s) detected at search point %zu, continuing search", 
            first_target.color.c_str(), i+1);
    }
    
    ROS_WARN("[Mission] Completed all search points, but no target with matching color (%s) found", first_target.color.c_str());
    return false;
}
// 发布最终目标航点
void publishFinalTarget() {
    if (second_target.color.empty()) {
        ROS_WARN("[Mission] No second search target found, cannot publish final target point");
        return;
    }
    
    final_target = second_target;
    
    // 创建最终航点
    Waypoint final_wp;
    final_wp.x = final_target.position.x();
    final_wp.y = final_target.position.y();
    final_wp.z = 0.95;  // 固定高度1.2米
    final_wp.yaw = 0.0;
    final_wp.delay = waypoint_timeout;
    
    ROS_INFO("[Mission] Publishing final target point: color=%s, position=(%.2f, %.2f, %.2f)",
         final_target.color.c_str(), final_wp.x, final_wp.y, final_wp.z);
    
    publishWaypoint(final_wp);
    waitForWaypointReached(final_wp, final_wp.delay);
    
    ROS_INFO("[Mission] Final target point reached");
}
// 执行抓球操作
bool executeBallGrab() {
    if (!use_ball_grab || first_target.color.empty()) {
        ROS_INFO("[Mission] Ball grab disabled or no target found, skipping");
        return false;
    }
    
    ROS_INFO("[Mission] Executing ball grab sequence");
    
    // 计算抓球点坐标
    Waypoint grab_wp;
    grab_wp.x = first_target.position.x() + ball_to_target_offset.x() + fixed_grab_offset.x();
    grab_wp.y = first_target.position.y() + ball_to_target_offset.y() + fixed_grab_offset.y();
    grab_wp.z = ball_to_target_offset.z() + fixed_grab_offset.z(); // 高度是绝对高度
    grab_wp.yaw = 0.0;
    grab_wp.delay = waypoint_timeout;
    
    ROS_INFO("[Mission] Flying to ball grab point: (%.2f, %.2f, %.2f)", 
             grab_wp.x, grab_wp.y, grab_wp.z);
    
    // 飞向抓球位置
    publishWaypoint(grab_wp);
    bool reached = waitForWaypointReached(grab_wp, grab_wp.delay);
    
    if (!reached) {
        ROS_WARN("[Mission] Failed to reach ball grab point, continuing anyway");
    }
    
    // 发送命令闭合夹爪
    ROS_INFO("[Mission] Closing gripper to grab the ball");
    sendGripperCommand(true);  // true = 闭合
    
    // 等待2秒
    ROS_INFO("[Mission] Waiting 2 seconds after closing gripper");
    delay(2.0);
    
    return true;
}

// 执行放球操作
bool executeBallDrop() {
    if (!use_ball_drop || second_target.color.empty()) {
        ROS_INFO("[Mission] Ball drop disabled or no target found, skipping");
        return false;
    }
    
    ROS_INFO("[Mission] Executing ball drop sequence");
    
    // 计算放球点坐标
    Waypoint drop_wp;
    drop_wp.x = second_target.position.x() + drop_to_target_offset.x() + fixed_grab_offset.x();
    drop_wp.y = second_target.position.y() + drop_to_target_offset.y();
    drop_wp.z = drop_to_target_offset.z(); // 高度是绝对高度
    drop_wp.yaw = 0.0;
    drop_wp.delay = waypoint_timeout;
    
    ROS_INFO("[Mission] Flying to ball drop point: (%.2f, %.2f, %.2f)", 
             drop_wp.x, drop_wp.y, drop_wp.z);
    
    // 飞向放球位置
    publishWaypoint(drop_wp);
    bool reached = waitForWaypointReached(drop_wp, drop_wp.delay);
    
    if (!reached) {
        ROS_WARN("[Mission] Failed to reach ball drop point, continuing anyway");
    }
    
    // 发送命令张开夹爪
    ROS_INFO("[Mission] Opening gripper to drop the ball");
    sendGripperCommand(false);  // false = 张开
    
    // 等待2秒
    ROS_INFO("[Mission] Waiting 2 seconds after opening gripper");
    delay(2.0);
    
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh("~");

    // 初始化发布者
    takeoff_pub = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 1);
    waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    yaw_pub = nh.advertise<std_msgs::Float32>("/external_yaw", 1);
    enable_pub = nh.advertise<std_msgs::Empty>("/traj_server/enable_cmd", 1);
    disable_pub = nh.advertise<std_msgs::Empty>("/traj_server/stop_cmd", 1);
    detection_enable_pub = nh.advertise<std_msgs::Bool>("/target_detection/enable", 1);
    detection_mode_pub = nh.advertise<std_msgs::String>("/target_detection/mode", 1);

    // 初始化订阅者
    ros::Subscriber initial_pos_sub = nh.subscribe("/drone/initial_position", 1, initialPositionCallback);
    ros::Subscriber target_pos_sub = nh.subscribe("/target_detection/position", 10, targetPositionCallback);
    ros::Subscriber odom_sub = nh.subscribe("/Odom_high_freq", 10, odomCallback);

    // 读取起飞延时参数
    double takeoff_delay;
    nh.param<double>("takeoff_delay", takeoff_delay, 10.0);  // 默认10秒

    // 读取航点到达和超时参数
    nh.param<double>("position_threshold", position_threshold, 0.2);
    nh.param<double>("velocity_threshold", velocity_threshold, 0.3);
    nh.param<double>("arrival_duration", arrival_duration, 2.0);
    nh.param<double>("waypoint_timeout", waypoint_timeout, 25.0);
    
    ROS_INFO("[Mission] Waypoint arrival params: pos=%.2f m, vel=%.2f m/s, delta_t=%.1f s, timeout=%.1f s",
             position_threshold, velocity_threshold, arrival_duration, waypoint_timeout);

    // 等待获取初始位置
    ROS_INFO("[Mission] Waiting initial position...");
    ros::Rate rate(10);
    while (ros::ok() && !initial_position_received) {
        ros::spinOnce();
        rate.sleep();
    }
    
    if (!initial_position_received) {
        ROS_ERROR("[Mission] Failed to get initial position, mission aborted");
        return -1;
    }
    
    // 等待获取里程计数据
    ROS_INFO("[Mission] Waiting odom data...");
    while (ros::ok() && !odom_received) {
        ros::spinOnce();
        rate.sleep();
    }
    
    if (!odom_received) {
        ROS_ERROR("[Mission] Failed to get odometry data, mission aborted");
        return -1;
    }

    // 读取主航点
    std::vector<std::vector<double>> main_waypoints;
    int waypoint_count = 0;
    
    // 从参数服务器读取航点数量
    nh.param<int>("waypoint_count", waypoint_count, 0);
    
    // 读取每个航点的参数
    for(int i = 0; i < waypoint_count; i++) {
        std::vector<double> wp(3);
        std::string prefix = "waypoints/" + std::to_string(i) + "/";
        
        nh.param<double>(prefix + "x", wp[0], 0.0);
        nh.param<double>(prefix + "y", wp[1], 0.0);
        nh.param<double>(prefix + "z", wp[2], 1.0);  // 默认高度1.0米
        
        main_waypoints.push_back(wp);
        
        ROS_INFO("[Mission] Loaded main waypoint %d: (%.2f, %.2f, %.2f)",
            i, wp[0], wp[1], wp[2]);
    }
    
    // 读取搜索点
    std::vector<std::vector<double>> search_waypoints;
    int search_count = 0;
    
    // 从参数服务器读取搜索点数量
    nh.param<int>("search_count", search_count, 0);
    
    // 读取每个搜索点的参数
    for(int i = 0; i < search_count; i++) {
        std::vector<double> wp(3);
        std::string prefix = "search_points/" + std::to_string(i) + "/";
        
        nh.param<double>(prefix + "x", wp[0], 0.0);
        nh.param<double>(prefix + "y", wp[1], 0.0);
        wp[2] = 1.2;  // 固定搜索高度
        
        search_waypoints.push_back(wp);
        
        ROS_INFO("[Mission] Loaded search point %d: (%.2f, %.2f, %.2f)",
         i, wp[0], wp[1], wp[2]);
    }
    
    // 读取搜索高度和检测时间
    double search_height, detection_time;
    nh.param<double>("search_height", search_height, 1.2);  // 默认搜索高度1.2米
    nh.param<double>("detection_time", detection_time, 2.0);  // 默认检测时间2秒
    
    // 读取第二次搜索点
    std::vector<std::vector<double>> second_search_points;
    int search2_count = 0;
    
    // 从参数服务器读取第二次搜索点数量
    nh.param<int>("search2_count", search2_count, 0);
    
    // 读取每个第二次搜索点的参数
    for(int i = 0; i < search2_count; i++) {
        std::vector<double> wp(3);
        std::string prefix = "search2_points/" + std::to_string(i) + "/";
        
        nh.param<double>(prefix + "x", wp[0], 0.0);
        nh.param<double>(prefix + "y", wp[1], 0.0);
        wp[2] = 1.2;  // 固定搜索高度
        
        second_search_points.push_back(wp);
        
        ROS_INFO("[Mission] Loaded second search point %d: (%.2f, %.2f, %.2f)",
         i, wp[0], wp[1], wp[2]);
    }

    // 添加抓球和放球参数
    nh.param<bool>("use_ball_grab", use_ball_grab, false);
    nh.param<bool>("use_ball_drop", use_ball_drop, false);
    
    // 读取小球相对于目标的偏移参数
    double ball_offset_x, ball_offset_y, ball_offset_z;
    nh.param<double>("ball_to_target/x", ball_offset_x, -0.5);
    nh.param<double>("ball_to_target/y", ball_offset_y, 0.0);
    nh.param<double>("ball_to_target/z", ball_offset_z, 1.03);
    ball_to_target_offset = Eigen::Vector3d(ball_offset_x, ball_offset_y, ball_offset_z);
    
    // 读取固定抓取偏移参数
    double fixed_offset_x, fixed_offset_y, fixed_offset_z;
    nh.param<double>("fixed_grab_offset/x", fixed_offset_x, 0.016);
    nh.param<double>("fixed_grab_offset/y", fixed_offset_y, 0.0);
    nh.param<double>("fixed_grab_offset/z", fixed_offset_z, 0.05);
    fixed_grab_offset = Eigen::Vector3d(fixed_offset_x, fixed_offset_y, fixed_offset_z);
    
    // 读取放球相对于目标的偏移参数
    double drop_offset_x, drop_offset_y, drop_offset_z;
    nh.param<double>("drop_to_target/x", drop_offset_x, 0.5);
    nh.param<double>("drop_to_target/y", drop_offset_y, 0.0);
    nh.param<double>("drop_to_target/z", drop_offset_z, 0.4);
    drop_to_target_offset = Eigen::Vector3d(drop_offset_x, drop_offset_y, drop_offset_z);
    
    // 初始化夹爪串口
    if (use_ball_grab || use_ball_drop) {
        std::string serial_port;
        nh.param<std::string>("gripper_serial_port", serial_port, "/dev/ttyUSB0");
        if (!initGripperSerial(serial_port)) {
            ROS_WARN("[Mission] Failed to initialize gripper serial port, gripper commands will be skipped");
        }
    }

    // 启用规划器
    enablePlanner();
    delay(1.0);
    
    // 发送起飞命令
    sendTakeoff();
    ROS_INFO("[Mission] Waiting for takeoff (%.1f seconds)...", takeoff_delay);
    delay(takeoff_delay);  // 起飞后的延时保持不变
    
    // 执行前3个主航点
    for(int i = 0; i < 5 && i < main_waypoints.size(); i++) {
        Waypoint wp = transformToRelativeWaypoint(
            main_waypoints[i][0], 
            main_waypoints[i][1], 
            main_waypoints[i][2], 
            0.0,  // yaw
            waypoint_timeout  // 使用超时参数
        );
        
        ROS_INFO("[Mission] Flying to main waypoint %d/%d: (%.2f, %.2f, %.2f)",
        i+1, (int)main_waypoints.size(), 
        main_waypoints[i][0], main_waypoints[i][1], main_waypoints[i][2]);
                
        publishWaypoint(wp);
        waitForWaypointReached(wp, wp.delay);
    }
    
    // 第一次小范围目标搜索
    ROS_INFO("[Mission] Starting first small-range target search");
    bool first_search_result = executeFirstTargetSearch(search_waypoints, search_height, detection_time);
    
    // 若第一次搜索成功，执行抓球操作
    if (first_search_result) {
        // 执行抓球序列
        if (use_ball_grab) {
            executeBallGrab();
        }
    }
    
    // 执行后续的第4, 5, 6个航点
    for(int i = 5; i < 10 && i < main_waypoints.size(); i++) {
        Waypoint wp = transformToRelativeWaypoint(
            main_waypoints[i][0], 
            main_waypoints[i][1], 
            main_waypoints[i][2], 
            0.0,  // yaw
            waypoint_timeout  // 使用超时参数
        );
        
        ROS_INFO("[Mission] Flying to main waypoint %d/%d: (%.2f, %.2f, %.2f)",
        i+1, (int)main_waypoints.size(), 
        main_waypoints[i][0], main_waypoints[i][1], main_waypoints[i][2]);
                
        publishWaypoint(wp);
        waitForWaypointReached(wp, wp.delay);
    }
    
    // 第二次搜索 - 使用从参数服务器读取的航点
    if (main_waypoints.size() >= 10 && first_search_result) {
        // 使用从参数读取的第二次搜索点
        if (second_search_points.empty()) {
            ROS_WARN("[Mission] Second search waypoints not configured, skipping second search");
        } else {
            ROS_INFO("[Mission] Starting second target search, total search points: %zu", second_search_points.size());
            
            // 确保第二次搜索使用多目标模式
            bool second_search_result = executeSecondTargetSearch(second_search_points, search_height, detection_time);
            
            // 如果第二次搜索成功，先执行放球操作，再前往最终目标
            if (second_search_result) {
                // 执行放球序列
                if (use_ball_drop) {
                    executeBallDrop();
                }
                
                // 前往最终目标点
                publishFinalTarget();
            }
        }
    } else {
        ROS_WARN("[Mission] Skipping second search - conditions not met");
    }
    
    // 关闭串口
    if (gripper_serial_fd >= 0) {
        close(gripper_serial_fd);
        ROS_INFO("[Mission] Gripper serial port closed");
    }
    // 完成所有航点后降落
    ROS_INFO("[Mission] All tasks completed. Landing...");
    disablePlanner();
    delay(1.0);
    sendLand();
    
    ROS_INFO("[Mission] Mission completed");
    ros::spin();
    
    return 0;
}