#include "rclcpp/rclcpp.hpp"
#include "custom_pkg/srv/hss.hpp"                                        // CHANGE

#include <memory>

void add(const std::shared_ptr<custom_pkg::srv::Hss::Request> request,     
          std::shared_ptr<custom_pkg::srv::Hss::Response>       response)  
{
  response->sum = request->a + request->b + request->c;  
  // Compute the sum of three integers and store it in the response
                                  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld" " c: %ld",  
                request->a, request->b, request->c);                                         
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_ints_server");   
  // Create a node named "add_three_ints_server"

  rclcpp::Service<custom_pkg::srv::Hss>::SharedPtr service =               
    node->create_service<custom_pkg::srv::Hss>("add_three_ints",  &add);   

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three ints.");                     

  rclcpp::spin(node);
  rclcpp::shutdown();
}