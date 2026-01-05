#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

class PreApproachV2 : public rclcpp::Node {
public:
  PreApproachV2() : Node("pre_approach_v2") {
    final_approach_ = declare_parameter("final_approach", true);

    client_ = create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

    timer_ = create_wall_timer(std::chrono::seconds(2),
                               std::bind(&PreApproachV2::tryService, this));

    RCLCPP_INFO(get_logger(), "Waiting to trigger final approach service...");
  }

private:
  void tryService() {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Service not available yet...");
      return;
    }

    auto request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    request->attach_to_shelf = final_approach_;

    auto future = client_->async_send_request(
        request, std::bind(&PreApproachV2::responseCallback, this, _1));
  }

  void responseCallback(
      rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) {
    if (future.get()->complete) {
      RCLCPP_INFO(get_logger(), "Final approach completed successfully");
      timer_->cancel(); // stop retrying
    } else {
      RCLCPP_WARN(get_logger(), "Shelf not detected yet, retrying...");
    }
  }

  bool final_approach_;
  rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachV2>());
  rclcpp::shutdown();
  return 0;
}
