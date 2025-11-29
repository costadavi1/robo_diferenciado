#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// CHANGE 1: Include TwistStamped instead of Twist
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class StampedRobotDriver : public rclcpp::Node
{
public:
  StampedRobotDriver()
  : Node("stamped_driver_node")
  {
    // 1. TOPIC NAME
    // When use_stamped_vel is true, the controller usually listens on 'cmd_vel'
    // Check 'ros2 topic list' to be sure!
    std::string topic_name = "/diff_drive_controller/cmd_vel";

    // CHANGE 2: Create a Publisher for TwistStamped
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(topic_name, 10);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&StampedRobotDriver::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Stamped Driver Started. Publishing to: %s", topic_name.c_str());
  }

private:
  void timer_callback()
  {
    // CHANGE 3: Use TwistStamped message type
    auto message = geometry_msgs::msg::TwistStamped();

    // CHANGE 4: Fill the Header (Required for Stamped messages)
    // The controller checks the timestamp to ensure the command isn't "old"
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "base_link"; // Or 'chassis', whatever your robot root is

    // CHANGE 5: Fill the 'twist' part of the message
    message.twist.linear.x = 0.5;   // Forward
    message.twist.angular.z = 0.5;  // Turn

    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StampedRobotDriver>());
  rclcpp::shutdown();
  return 0;
}
