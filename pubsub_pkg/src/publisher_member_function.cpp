#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_pkg/msg/hss.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0) // Set node name and initialize counter
  {
    publisher_ = this->create_publisher<custom_pkg::msg::Hss>("topic", 10); // Create a publisher
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this)); // Call timer_callback every 500ms
  }

private:
  void timer_callback()
  {
    auto message = custom_pkg::msg::Hss();                                         // Create message
    message.num = this->count_++;                                                  // Set message data
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'"); // Log
    publisher_->publish(message);                                                  // Publish message
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_pkg::msg::Hss>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}