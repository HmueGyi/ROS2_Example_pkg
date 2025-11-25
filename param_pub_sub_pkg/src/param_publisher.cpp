#include <chrono>
#include <functional>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "custom_pkg/msg/hss.hpp"

using namespace std::chrono_literals;

class MinimalParam : public rclcpp::Node
{
public:
  MinimalParam()
  : Node("param_publisher_node" ),count_(0) 
  {
    publisher_ = this->create_publisher<custom_pkg::msg::Hss>("topic", 10); // Create a publisher
    this->declare_parameter("my_parameter", 1);

    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalParam::timer_callback, this));
  }

  void timer_callback()
  {
    int my_param = this->get_parameter("my_parameter").as_int();                                     
    
    custom_pkg::msg::Hss message;               //create message
    message.num = my_param;   
                                                      
    // RCLCPP_INFO(this->get_logger(), "%d", my_param);

    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: '" << message.num << "'");
    publisher_->publish(message);
    
    // my_param++;
    // this->set_parameter(rclcpp::Parameter("my_parameter", my_param));
    
    // std::vector<rclcpp::Parameter> all_new_parameters{rclcpp::Parameter("my_parameter", my_param )};
    // this->set_parameters(all_new_parameters);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<custom_pkg::msg::Hss>::SharedPtr publisher_;     // for publisher
  size_t count_; // Counter for the number of meclearssages published
}
;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalParam>());
  rclcpp::shutdown();
  return 0;
}