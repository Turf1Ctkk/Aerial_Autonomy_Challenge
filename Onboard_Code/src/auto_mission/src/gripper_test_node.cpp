// gripper_test_node.cpp
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

// Global variables
int gripper_serial_fd = -1;
std::string serial_port = "/dev/ttyUSB0";

// Initialize gripper serial port
bool initGripperSerial(const std::string& port_name) {
    // Open serial device
    gripper_serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);
    if (gripper_serial_fd < 0) {
        ROS_ERROR("Failed to open gripper serial port %s", port_name.c_str());
        return false;
    }

    // Set serial parameters
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    
    // Get current settings
    if (tcgetattr(gripper_serial_fd, &tty) != 0) {
        ROS_ERROR("Failed to get serial port attributes");
        close(gripper_serial_fd);
        gripper_serial_fd = -1;
        return false;
    }
    
    // Set baud rate (115200)
    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    // 8N1 (8 data bits, no parity, 1 stop bit)
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    
    // No flow control
    tty.c_cflag &= ~CRTSCTS;
    
    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    
    // Set new attributes
    if (tcsetattr(gripper_serial_fd, TCSANOW, &tty) != 0) {
        ROS_ERROR("Failed to set serial port attributes");
        close(gripper_serial_fd);
        gripper_serial_fd = -1;
        return false;
    }
    
    ROS_INFO("Gripper serial port initialized successfully: %s", port_name.c_str());
    return true;
}

// Send control command to gripper
bool sendGripperCommand(bool close_gripper) {
    if (gripper_serial_fd < 0) {
        ROS_WARN("Gripper serial port not initialized, cannot send command");
        return false;
    }
    
    // Prepare hex command to send
    unsigned char cmd[4];
    if (close_gripper) {
        // Close gripper command: ff fe bb ee
        cmd[0] = 0xff;
        cmd[1] = 0xfe;
        cmd[2] = 0xbb;
        cmd[3] = 0xee;
        ROS_INFO("Sending gripper CLOSE command: ff fe bb ee");
    } else {
        // Open gripper command: ff fe aa ee
        cmd[0] = 0xff;
        cmd[1] = 0xfe;
        cmd[2] = 0xaa;
        cmd[3] = 0xee;
        ROS_INFO("Sending gripper OPEN command: ff fe aa ee");
    }
    
    // Send command to serial port
    int result = write(gripper_serial_fd, cmd, 4);
    
    if (result < 0) {
        ROS_ERROR("Failed to send gripper command");
        return false;
    } else {
        ROS_INFO("Gripper command sent successfully (%d bytes)", result);
        return true;
    }
}

// Command callback function
void commandCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == "open") {
        sendGripperCommand(false);
    } else if (msg->data == "close") {
        sendGripperCommand(true);
    } else {
        ROS_WARN("Unknown command: %s (use 'open' or 'close')", msg->data.c_str());
    }
}

// Control service callback
bool controlServiceCallback(std_srvs::SetBool::Request &req, 
                            std_srvs::SetBool::Response &res) {
    bool success = sendGripperCommand(req.data);
    
    if (success) {
        res.success = true;
        res.message = req.data ? "Gripper closed" : "Gripper opened";
    } else {
        res.success = false;
        res.message = "Failed to send gripper command";
    }
    
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gripper_test_node");
    ros::NodeHandle nh("~");
    
    // Get parameters
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    
    // Initialize serial port
    if (!initGripperSerial(serial_port)) {
        ROS_ERROR("Gripper test node initialization failed!");
        return 1;
    }
    
    // Subscribe to control commands
    ros::Subscriber cmd_sub = nh.subscribe("command", 10, commandCallback);
    
    // Provide control service (true=close gripper, false=open gripper)
    ros::ServiceServer control_service = 
        nh.advertiseService("control", controlServiceCallback);
    
    ROS_INFO("Gripper test node started");
    ROS_INFO("Usage:");
    ROS_INFO("1. Publish messages to '~command' topic (content: 'open' or 'close')");
    ROS_INFO("2. Call '~control' service (true=close gripper, false=open gripper)");

    ros::spin();
    
    // Close serial port
    if (gripper_serial_fd >= 0) {
        close(gripper_serial_fd);
        ROS_INFO("Gripper serial port closed");
    }
    
    return 0;
}