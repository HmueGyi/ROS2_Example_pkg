// File: src/cycle/draw_cycle.cpp

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DrawCycle : public rclcpp::Node
{
public:
  // Constructor: runs when the node starts
  DrawCycle() : Node("draw_cycle_node")
  {
    // Create a publisher to send velocity commands to the turtle
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10); //twist for velecity commands

    // Create a timer to call publish_cmd() every 100 ms
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DrawCycle::publish_cmd, this));
  }

private:
  // This function sends velocity commands to make the turtle move in a circle
  void publish_cmd()
  {
    auto msg = geometry_msgs::msg::Twist();

    msg.linear.x = 2.0;  // Move forward at speed 2.0
    msg.angular.z = 1.0; // Rotate at speed 1.0 (to create circular motion)

    // Publish the command
    publisher_->publish(msg);

    // Print info to the console
    RCLCPP_INFO(this->get_logger(), "Drawing cycle (circle)...");
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DrawCycle>());
  rclcpp::shutdown();
  return 0;
}
