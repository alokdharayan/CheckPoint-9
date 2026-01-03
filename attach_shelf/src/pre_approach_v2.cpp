#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class PreApproachV2 : public rclcpp::Node {
public:
  PreApproachV2() : Node("pre_approach_v2") {
    final_approach_ = this->declare_parameter("final_approach", true);

    client_ =
        this->create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");

    timer_ = this->create_wall_timer(
        2s, std::bind(&PreApproachV2::callService, this));
  }

private:
  void callService() {
    if (!client_->wait_for_service(1s))
      return;

    auto req = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
    req->attach_to_shelf = final_approach_;

    client_->async_send_request(req);
    timer_->cancel();
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
