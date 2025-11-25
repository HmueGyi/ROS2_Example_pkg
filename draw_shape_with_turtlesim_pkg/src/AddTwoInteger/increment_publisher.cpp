#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class IncrementPublisher : public rclcpp::Node
{
public:
    // This runs when the node starts
    IncrementPublisher() : Node("increment_publisher"), counter_(0)
    {
        // Create a publisher that sends numbers on "increment_topic"
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("increment_topic", 10);

        // Set a timer to call the timer_callback() every 1 second
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&IncrementPublisher::timer_callback, this));
    }

private:
    // This function runs every 1 second
    void timer_callback()
    {
        // Create a message
        auto msg = std_msgs::msg::Int32();

        // Put the current number into the message, then increase it
        msg.data = counter_++;

        // Print the number to the terminal
        RCLCPP_INFO(this->get_logger(), "Publishing: %d", msg.data);

        // Send the message
        publisher_->publish(msg);
    }

    // A publisher to send messages
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    // A timer to call the function every second
    rclcpp::TimerBase::SharedPtr timer_;

    // A number that goes up every second
    int counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IncrementPublisher>());
    rclcpp::shutdown();
    return 0;
}
