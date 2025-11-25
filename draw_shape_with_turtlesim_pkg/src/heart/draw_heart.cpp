// #include <cmath>
// #include <memory>
// #include <vector>
// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "turtlesim/msg/pose.hpp"

// using namespace std::chrono_literals;

// class HeartDrawer : public rclcpp::Node
// {
// public:
//   HeartDrawer()
//   : Node("heart_drawer"), index_(0)
//   {
//     pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
//     sub_ = this->create_subscription<turtlesim::msg::Pose>(
//       "turtle1/pose", 10, std::bind(&HeartDrawer::pose_callback, this, std::placeholders::_1));

//     // Bigger heart scale
//     double scale = 0.08;  // doubled from 0.04 to make it bigger

//     for (double t = 0; t <= 2 * M_PI; t += 0.1) {
//       double x = 16 * std::pow(std::sin(t), 3);
//       double y = 13 * std::cos(t) - 5 * std::cos(2*t) - 2 * std::cos(3*t) - std::cos(4*t);
//       points_.emplace_back(x * scale + 5.5, y * scale + 5.5);
//     }
//   }

// private:
//   void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
//   {
//     if (index_ >= points_.size()) {
//       auto stop_msg = geometry_msgs::msg::Twist();
//       pub_->publish(stop_msg);
//       RCLCPP_INFO(this->get_logger(), "Finished drawing heart.");
//       rclcpp::shutdown();
//       return;
//     }

//     double goal_x = points_[index_].first;
//     double goal_y = points_[index_].second;

//     double dx = goal_x - msg->x;
//     double dy = goal_y - msg->y;
//     double distance = std::hypot(dx, dy);

//     double angle_to_goal = std::atan2(dy, dx);
//     double angle_diff = angle_to_goal - msg->theta;

//     // Normalize angle_diff to [-pi, pi]
//     while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
//     while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

//     geometry_msgs::msg::Twist cmd;

//     const double angle_tolerance = 0.05;
//     const double distance_tolerance = 0.05;

//     // Faster angular speed multiplier
//     const double angular_speed_gain = 5.0;

//     // Faster linear speed multiplier
//     const double linear_speed_gain = 3.5;

//     if (distance > distance_tolerance) {
//       if (std::fabs(angle_diff) > angle_tolerance) {
//         // Turn faster
//         cmd.linear.x = 0.0;
//         cmd.angular.z = angular_speed_gain * angle_diff;
//       } else {
//         // Move faster forward
//         cmd.linear.x = linear_speed_gain * distance;
//         cmd.angular.z = 0.0;
//       }
//     } else {
//       index_++;
//     }

//     pub_->publish(cmd);
//   }

//   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
//   rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
//   std::vector<std::pair<double, double>> points_;
//   size_t index_;
// };

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<HeartDrawer>());
//   rclcpp::shutdown();
//   return 0;
// }



#include <cmath>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::chrono_literals;

class HeartDrawer : public rclcpp::Node
{
public:
  HeartDrawer()
  : Node("heart_drawer"), index_(0)
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "turtle1/pose", 10, std::bind(&HeartDrawer::pose_callback, this, std::placeholders::_1));

    double scale = 0.15;  // bigger heart

    for (double t = 0; t <= 2 * M_PI; t += 0.1) {
      double x = 16 * std::pow(std::sin(t), 3);
      double y = 13 * std::cos(t) - 5 * std::cos(2*t) - 2 * std::cos(3*t) - std::cos(4*t);
      points_.emplace_back(x * scale + 5.5, y * scale + 5.5);
    }
  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    if (index_ >= points_.size()) {
      index_ = 0;  // loop infinitely
      RCLCPP_INFO(this->get_logger(), "Restarting heart drawing...");
    }

    double goal_x = points_[index_].first;
    double goal_y = points_[index_].second;

    double dx = goal_x - msg->x;
    double dy = goal_y - msg->y;
    double distance = std::hypot(dx, dy);

    double angle_to_goal = std::atan2(dy, dx);
    double angle_diff = angle_to_goal - msg->theta;

    while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

    geometry_msgs::msg::Twist cmd;

    const double angle_tolerance = 0.05;
    const double distance_tolerance = 0.05;

    const double angular_speed_gain = 15.0;  // faster turning
    const double linear_speed_gain = 12.5;   // faster forward

    if (distance > distance_tolerance) {
      if (std::fabs(angle_diff) > angle_tolerance) {
        cmd.linear.x = 0.0;
        cmd.angular.z = angular_speed_gain * angle_diff;
      } else {
        cmd.linear.x = linear_speed_gain * distance;
        cmd.angular.z = 0.0;
      }
    } else {
      index_++;
    }

    pub_->publish(cmd);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
  std::vector<std::pair<double, double>> points_;
  size_t index_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeartDrawer>());
  rclcpp::shutdown();
  return 0;
}
