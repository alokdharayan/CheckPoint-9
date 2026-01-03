#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

using std::placeholders::_1;

class PreApproach : public rclcpp::Node {
public:
  PreApproach() : Node("pre_approach") {
    obstacle_ = declare_parameter("obstacle", 0.3);
    degrees_ = declare_parameter("degrees", -90.0);

    // Correct QoS for laser data
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&PreApproach::scanCallback, this, _1));

    // Publish to generic cmd_vel (will be remapped)
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&PreApproach::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Pre-approach node started");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float d = msg->ranges[msg->ranges.size() / 2];

    // Ignore invalid Gazebo readings
    if (d > 0.05)
      front_distance_ = d;
  }

  void controlLoop() {
    geometry_msgs::msg::Twist cmd;

    if (!stopped_) {
      if (front_distance_ > obstacle_) {
        cmd.linear.x = 0.2;
      } else {
        stopped_ = true;
        start_time_ = now();
      }
    } else if (!rotated_) {
      double rad = degrees_ * M_PI / 180.0;
      double w = (rad > 0) ? 0.5 : -0.5;
      double t = std::abs(rad / w);

      if ((now() - start_time_).seconds() < t)
        cmd.angular.z = w;
      else
        rotated_ = true;
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time start_time_;
  double obstacle_, degrees_;
  float front_distance_ = 10.0;

  bool stopped_ = false;
  bool rotated_ = false;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproach>());
  rclcpp::shutdown();
  return 0;
}
