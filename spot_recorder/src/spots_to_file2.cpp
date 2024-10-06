#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spot_recorder/srv/my_service_message.hpp"

#include <memory>

class SpotRecorder : public rclcpp::Node {
public:
  SpotRecorder() : Node("spot_recorder") {

    cb_group_1_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_group_2_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_1_;

    // Configure QoS settings
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Create a subscription to the topic
    subscription_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "amcl_pose", qos,
        std::bind(&SpotRecorder::topic_callback, this, std::placeholders::_1),
        sub_options);

    // Create a service server
    service_ = this->create_service<spot_recorder::srv::MyServiceMessage>(
        "spot_recorder",
        std::bind(&SpotRecorder::handle_service, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_default, cb_group_2_);
  }

private:
  void topic_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    last_received_pose_ = msg->pose;
    RCLCPP_INFO(this->get_logger(), "Receive...");
    RCLCPP_INFO(this->get_logger(),
                "Received AMCL pose: [x: %f, y: %f, orientation: %f]",
                last_received_pose_.pose.position.x,
                last_received_pose_.pose.position.y,
                last_received_pose_.pose.orientation.w);
    // RCLCPP_INFO(this->get_logger(), "Received data: %s",
    // last_received_data_.c_str());
  }

  void handle_service(
      const std::shared_ptr<spot_recorder::srv::MyServiceMessage::Request>
          request,
      std::shared_ptr<spot_recorder::srv::MyServiceMessage::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "Handling service request...");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nlabel: %s",
                request->label.c_str());
    RCLCPP_INFO(this->get_logger(),
                "Received AMCL pose: [x: %f, y: %f, orientation: %f]",
                last_received_pose_.pose.position.x,
                last_received_pose_.pose.position.y,
                last_received_pose_.pose.orientation.w);
    // Use the last received data from the topic in the response
    // response->response_field = last_received_data_;
    // RCLCPP_INFO(this->get_logger(), "Responding with: %s",
    // response->response_field.c_str());
  }

  rclcpp::CallbackGroup::SharedPtr cb_group_1_;
  rclcpp::CallbackGroup::SharedPtr cb_group_2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      subscription_;
  rclcpp::Service<spot_recorder::srv::MyServiceMessage>::SharedPtr service_;
  geometry_msgs::msg::PoseWithCovariance last_received_pose_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpotRecorder>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
