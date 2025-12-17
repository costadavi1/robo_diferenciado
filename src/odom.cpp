#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;

class OdomNode : public rclcpp::Node
{
public:
  OdomNode()
      : Node("odom_node")
  {
    this->declare_parameter("wheel_radius", 0.0);
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();

    this->declare_parameter("wheel_separation", 0.0);
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();

    last_time_ = this->now();

    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&OdomNode::topic_callback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&OdomNode::lookup_tf, this));

    global_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/global", 10);
  }

private:
  void lookup_tf()
  {
    try
    {

      geometry_msgs::msg::TransformStamped transform =
          tf_buffer_->lookupTransform(
              "map",
              "base_link",
              tf2::TimePointZero);

      nav_msgs::msg::Odometry gt_odom;
      gt_odom.header.frame_id = "map";
      gt_odom.header.stamp = this->get_clock()->now();

      gt_odom.pose.pose.position.x = transform.transform.translation.x;
      gt_odom.pose.pose.position.y = transform.transform.translation.y;
      gt_odom.pose.pose.position.z = transform.transform.translation.z;

      gt_odom.pose.pose.orientation.x = transform.transform.rotation.x;
      gt_odom.pose.pose.orientation.y = transform.transform.rotation.y;
      gt_odom.pose.pose.orientation.z = transform.transform.rotation.z;
      gt_odom.pose.pose.orientation.w = transform.transform.rotation.w;

      global_odom_pub_->publish(gt_odom);
    }
    catch (const tf2::TransformException &ex)
    {
      // RCLCPP_INFO(
      //     this->get_logger(), "Could not transform: %s", ex.what());
      return;
    }
  }
  void topic_callback(const sensor_msgs::msg::JointState &msg)
  {
    static int idx_left = -1, idx_right = -1;

    if (idx_left < 0 || idx_right < 0)
    {
      for (size_t i = 0; i < msg.name.size(); i++)
      {
        if (msg.name[i] == "left_wheel_joint")
          idx_left = i;
        if (msg.name[i] == "right_wheel_joint")
          idx_right = i;
      }
      if (idx_left < 0 || idx_right < 0)
      {
        RCLCPP_WARN(this->get_logger(), "'left_wheel_joint' e 'right_wheel_joint' not found!");
        return;
      }
    }

    double left_pos = msg.position[idx_left];
    double right_pos = msg.position[idx_right];

    double dphi_left = left_pos - last_left_pos_;
    double dphi_right = right_pos - last_right_pos_;

    last_left_pos_ = left_pos;
    last_right_pos_ = right_pos;

    double ds_left = wheel_radius_ * dphi_left;
    double ds_right = wheel_radius_ * dphi_right;

    double ds = (ds_right + ds_left) / 2.0;
    double dtheta = (ds_right - ds_left) / wheel_separation_;

    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double theta_mid = theta_ + dtheta / 2.0;

    x_ += ds * cos(theta_mid);
    y_ += ds * sin(theta_mid);
    theta_ += dtheta;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Velocidades estimadas
    odom_msg.twist.twist.linear.x = ds / dt;
    odom_msg.twist.twist.angular.z = dtheta / dt;

    odom_pub_->publish(odom_msg);
  }

  double wheel_radius_, wheel_separation_;

  double last_left_pos_ = 0.0;
  double last_right_pos_ = 0.0;
  double x_ = 0.0, y_ = 0.0, theta_ = 0.0;

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomNode>());
  rclcpp::shutdown();
  return 0;
}