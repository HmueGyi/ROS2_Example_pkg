#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class AddFiveNode : public rclcpp::Node
{
public:
    // When the node starts
    AddFiveNode() : Node("add_five_number")
    {
        // Subscribe to "increment_topic" to receive numbers
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "increment_topic", 10,
            std::bind(&AddFiveNode::callback, this, std::placeholders::_1));

        // Create a publisher to send new numbers to "increment_plus_five_topic"
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("increment_plus_five_topic", 10);
    }

private:
    // Runs when a message is received
    void callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // Make a new message and add 5 to the received number
        auto new_msg = std_msgs::msg::Int32();
        new_msg.data = msg->data + 5;

        // Print the original and new number
        RCLCPP_INFO(this->get_logger(), "Received: %d, Publishing: %d", msg->data, new_msg.data);

        // Send the new number
        publisher_->publish(new_msg);
    }

    // This gets messages from "increment_topic"
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

    // This sends messages to "increment_plus_five_topic"
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AddFiveNode>());
    rclcpp::shutdown();
    return 0;
}
