#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_pkg/msg/hss.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
      : Node("minimal_subscriber") // Set node name
  {
    subscription_ = this->create_subscription<custom_pkg::msg::Hss>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    // Subscribe to "topic" with QoS 10, and call topic_callback when a message arrives
  }

private:
  void topic_callback(const custom_pkg::msg::Hss &msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg.num << "'");
    // Log the received message
  }
  rclcpp::Subscription<custom_pkg::msg::Hss>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}