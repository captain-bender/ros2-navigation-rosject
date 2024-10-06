#include "rclcpp/rclcpp.hpp"
#include "spot_recorder/srv/my_service_message.hpp"

#include <memory>

void record(
    const std::shared_ptr<spot_recorder::srv::MyServiceMessage::Request>
        request,
    std::shared_ptr<spot_recorder::srv::MyServiceMessage::Response> response) {
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nlabel: %s",
              request->label.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("spot_recorder_server");

  rclcpp::Service<spot_recorder::srv::MyServiceMessage>::SharedPtr service =
      node->create_service<spot_recorder::srv::MyServiceMessage>(
          "spot_recorder", &record);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to record spots.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}