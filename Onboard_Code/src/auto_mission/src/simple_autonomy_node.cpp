#include <ros/ros.h>

#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>

#include <cstdint>

class SimpleAutonomyNode
{
public:
  SimpleAutonomyNode()
      : state_(WAIT_TAKEOFF_SUBSCRIBER),
        active_cmd_(0),
        cmd_publish_count_(0),
        cmd_seen_(false)
  {
    takeoff_land_pub_ = nh_.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 1);
    cmd_sub_ = nh_.subscribe("/setpoints_cmd", 20, &SimpleAutonomyNode::cmdCallback, this,
                             ros::TransportHints().tcpNoDelay());
    timer_ = nh_.createTimer(ros::Duration(0.1), &SimpleAutonomyNode::timerCallback, this);
  }

private:
  enum State
  {
    WAIT_TAKEOFF_SUBSCRIBER,
    PUBLISH_TAKEOFF,
    WAIT_CMD_START,
    WAIT_CMD_STOP,
    PUBLISH_LAND,
    DONE
  };

  static constexpr uint8_t TAKEOFF_CMD = 1;
  static constexpr uint8_t LAND_CMD = 2;
  static constexpr double CMD_TIMEOUT = 0.8;
  static constexpr int COMMAND_PUBLISH_COUNT = 10;

  ros::NodeHandle nh_;
  ros::Publisher takeoff_land_pub_;
  ros::Subscriber cmd_sub_;
  ros::Timer timer_;

  ros::Time last_cmd_time_;
  State state_;
  uint8_t active_cmd_;
  int cmd_publish_count_;
  bool cmd_seen_;

  void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr &)
  {
    last_cmd_time_ = ros::Time::now();
    cmd_seen_ = true;
  }

  void timerCallback(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();

    switch (state_)
    {
    case WAIT_TAKEOFF_SUBSCRIBER:
      if (takeoff_land_pub_.getNumSubscribers() > 0)
      {
        startCommand(TAKEOFF_CMD);
        state_ = PUBLISH_TAKEOFF;
        ROS_INFO("[simple_autonomy] px4ctrl subscriber connected. Publishing takeoff command.");
      }
      break;

    case PUBLISH_TAKEOFF:
      if (publishCommandBurst())
      {
        state_ = WAIT_CMD_START;
        ROS_INFO("[simple_autonomy] Takeoff command published.");
      }
      break;

    case WAIT_CMD_START:
      if (cmd_seen_)
      {
        state_ = WAIT_CMD_STOP;
        ROS_INFO("[simple_autonomy] Position command received. Waiting for command timeout.");
      }
      break;

    case WAIT_CMD_STOP:
      if ((now - last_cmd_time_).toSec() > CMD_TIMEOUT)
      {
        startCommand(LAND_CMD);
        state_ = PUBLISH_LAND;
        ROS_INFO("[simple_autonomy] Position command timed out. Publishing land command.");
      }
      break;

    case PUBLISH_LAND:
      if (publishCommandBurst())
      {
        state_ = DONE;
        ROS_INFO("[simple_autonomy] Land command published. Exit.");
        ros::shutdown();
      }
      break;

    case DONE:
      break;
    }
  }

  void startCommand(uint8_t cmd)
  {
    active_cmd_ = cmd;
    cmd_publish_count_ = 0;
  }

  bool publishCommandBurst()
  {
    publishTakeoffLand(active_cmd_);
    cmd_publish_count_++;
    return cmd_publish_count_ >= COMMAND_PUBLISH_COUNT;
  }

  void publishTakeoffLand(uint8_t cmd)
  {
    quadrotor_msgs::TakeoffLand msg;
    msg.takeoff_land_cmd = cmd;
    takeoff_land_pub_.publish(msg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_autonomy");
  SimpleAutonomyNode node;
  ros::spin();
  return 0;
}
