#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class PreApproach : public rclcpp::Node
{
public:
  PreApproach() : Node("pre_approach"), stopped_(false), rotated_(false)
  {
    obstacle_ = this->declare_parameter("obstacle", 0.3);
    degrees_  = this->declare_parameter("degrees", -90.0);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&PreApproach::scanCallback, this, _1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/robot/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PreApproach::controlLoop, this));
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int center = msg->ranges.size() / 2;
    front_dist_ = msg->ranges[center];
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist cmd;

    if (!stopped_)
    {
      if (front_dist_ > obstacle_)
      {
        cmd.linear.x = 0.2;
      }
      else
      {
        stopped_ = true;
        start_time_ = now();
      }
    }
    else if (!rotated_)
    {
      double rad = degrees_ * M_PI / 180.0;
      double ang_vel = (rad > 0) ? 0.5 : -0.5;
      double duration = std::abs(rad / ang_vel);

      if ((now() - start_time_).seconds() < duration)
      {
        cmd.angular.z = ang_vel;
      }
      else
      {
        rotated_ = true;
      }
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  double obstacle_, degrees_;
  float front_dist_{10.0};

  bool stopped_, rotated_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
