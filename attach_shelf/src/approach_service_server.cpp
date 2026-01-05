#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>

using std::placeholders::_1;
using std::placeholders::_2;

class ApproachService : public rclcpp::Node {
public:
  ApproachService() : Node("approach_service_server") {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ApproachService::scanCallback, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);

    lift_pub_ = create_publisher<std_msgs::msg::Empty>("/elevator_up", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    service_ = create_service<attach_shelf::srv::GoToLoading>(
        "/approach_shelf",
        std::bind(&ApproachService::serviceCallback, this, _1, _2));

    // 🔑 Timer to continuously publish TF
    tf_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                  std::bind(&ApproachService::publishTF, this));
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_ = msg;
  }

  void
  serviceCallback(const attach_shelf::srv::GoToLoading::Request::SharedPtr req,
                  attach_shelf::srv::GoToLoading::Response::SharedPtr res) {
    if (!scan_) {
      res->complete = false;
      return;
    }

    std::vector<int> peaks;
    for (size_t i = 0; i < scan_->intensities.size(); i++)
      if (scan_->intensities[i] > 5000)
        peaks.push_back(i);

    if (peaks.size() < 2) {
      res->complete = false;
      return;
    }

    int i1 = peaks.front();
    int i2 = peaks.back();

    double a1 = scan_->angle_min + i1 * scan_->angle_increment;
    double a2 = scan_->angle_min + i2 * scan_->angle_increment;

    double r1 = scan_->ranges[i1];
    double r2 = scan_->ranges[i2];

    cart_x_ = (r1 * cos(a1) + r2 * cos(a2)) / 2.0;
    cart_y_ = (r1 * sin(a1) + r2 * sin(a2)) / 2.0;
    cart_detected_ = true;

    if (req->attach_to_shelf) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = 0.2;
      cmd_pub_->publish(cmd);
      rclcpp::sleep_for(std::chrono::seconds(2));

      cmd.linear.x = 0.0;
      cmd_pub_->publish(cmd);

      lift_pub_->publish(std_msgs::msg::Empty());
    }

    res->complete = true;
  }

  // 🔑 Continuous TF broadcaster
  void publishTF() {
    if (!cart_detected_)
      return;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "robot_base_link";
    tf.child_frame_id = "cart_frame";

    tf.transform.translation.x = cart_x_;
    tf.transform.translation.y = cart_y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(tf);
  }

  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr lift_pub_;
  rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  double cart_x_{0.0}, cart_y_{0.0};
  bool cart_detected_{false};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachService>());
  rclcpp::shutdown();
  return 0;
}
