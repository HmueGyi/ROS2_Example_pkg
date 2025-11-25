#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class FinalSubscriber : public rclcpp::Node
{
public:
    // Constructor: runs when the node starts
    FinalSubscriber() : Node("final_subscriber")
    {
        // Subscribe to "increment_plus_five_topic" to receive numbers
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "increment_plus_five_topic", 10,
            std::bind(&FinalSubscriber::callback, this, std::placeholders::_1));
    }

private:
    // This function runs every time a message is received
    void callback(const std_msgs::msg::Int32::SharedPtr msg) const
    {
        // Print the received number
        RCLCPP_INFO(this->get_logger(), "Final received value: %d", msg->data);
    }

    // Subscription to receive messages from the topic
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FinalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
