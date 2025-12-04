#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <nav_msgs/msg/path.hpp>

#include "robo_diferenciado/action/follow_path.hpp"

class PathFollower : public rclcpp::Node
{
public:
  using FollowPath = robo_diferenciado::action::FollowPath;
  using GoalHandlePathFollower = rclcpp_action::ServerGoalHandle<FollowPath>;

  PathFollower()
      : Node("path_follower")
  {
    using namespace std::placeholders;
    this->declare_parameter("lookahead_distance", 0.3);
    lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();

    // Create subscriber for robot pose (odometry)
    this->robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/diff_drive_controller/odom", 10,
        std::bind(&PathFollower::robot_pose_callback, this, _1));
    // Create publisher for cmd_vel
    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);
    // Create action server
    this->action_server_ = rclcpp_action::create_server<FollowPath>(this,
        "follow_path",
        std::bind(&PathFollower::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PathFollower::handle_cancel, this, std::placeholders::_1),
        std::bind(&PathFollower::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const FollowPath::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;

      // VALIDATION CHECK 1: Is the path empty?
    if (goal->path.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Goal rejected: Path is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // VALIDATION CHECK 2: Check frame_id
    if (goal->path.header.frame_id != "odom" && goal->path.header.frame_id != "map") {
      RCLCPP_WARN(this->get_logger(), "Goal rejected: Frame must be odom or map");
      // return rclcpp_action::GoalResponse::REJECT; // Uncomment to enforce
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      std::shared_ptr<GoalHandlePathFollower> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    auto cmd_msg = geometry_msgs::msg::TwistStamped();

    // Send command to stop the robot
    cmd_msg.twist.linear.x = 0.0;
    cmd_msg.twist.angular.z = 0.0;
    cmd_msg.header.stamp = this->now();

    cmd_vel_pub_->publish(cmd_msg);

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      std::shared_ptr<GoalHandlePathFollower> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    std::thread{std::bind(&PathFollower::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePathFollower> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    // Implement path following logic here
    auto feedback = std::make_shared<FollowPath::Feedback>();
    auto result = std::make_shared<FollowPath::Result>();
    auto cmd_msg = geometry_msgs::msg::TwistStamped();
    const auto goal = goal_handle->get_goal();
    rclcpp::Rate loop_rate(10);

    // Set initial waypoint as the robot's current position
    geometry_msgs::msg::Pose current_waypoint = this->robot_pose_;

    size_t n_wpts = goal->path.poses.size();
    for (size_t i = 0; i < n_wpts; i++)
    {
      // Get the next waypoint
      geometry_msgs::msg::Pose next_waypoint = goal->path.poses[i].pose;
      // Calculate distance between current and next waypoints
      double dx_waypoints = next_waypoint.position.x - current_waypoint.position.x;
      double dy_waypoints = next_waypoint.position.y - current_waypoint.position.y;
      double distance_waypoints = std::sqrt(dx_waypoints * dx_waypoints + dy_waypoints * dy_waypoints);

      // Create intermediate waypoint for lookahead
      auto intermediate_waypoint = current_waypoint;
      do
      {
        double dx = robot_pose_.position.x - next_waypoint.position.x;
        double dy = robot_pose_.position.y - next_waypoint.position.y;
        double distance = std::sqrt(dx * dx + dy * dy_waypoints);

        // Check if waypoint is reached or within lookahead distance
        if (distance < distance_threshold_ || distance < lookahead_distance_)
        {
          RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", i);
          break;
        }

        // At the current implementation, this transformation is not necessaryly,
        // but it is included here to illustrate how to transform global to local frames.
        auto intermediate_wpt_body_frame = transformGlobalToLocal(robot_pose_, intermediate_waypoint);
        auto distance_to_intermediate = std::sqrt(
            intermediate_wpt_body_frame.position.x * intermediate_wpt_body_frame.position.x +
            intermediate_wpt_body_frame.position.y * intermediate_wpt_body_frame.position.y);

        RCLCPP_INFO(this->get_logger(), "Distance to Intermediate: %.2f", distance_to_intermediate);

        if (distance_to_intermediate >= lookahead_distance_)
        {
          double ratio = lookahead_distance_ / distance_to_intermediate;
          double lookahead_x = intermediate_wpt_body_frame.position.x * ratio;
          double lookahead_y = intermediate_wpt_body_frame.position.y * ratio;
          RCLCPP_DEBUG(this->get_logger(), "Lookahead Point: x=%.2f, y=%.2f", lookahead_x, lookahead_y);

          // Simple pure pursuit control law (for demonstration)
          double k_linear = 0.5;  // Gain for linear velocity
          double k_angular = 1.0; // Gain for angular velocity

          cmd_msg.twist.linear.x = k_linear * distance_to_intermediate;

          cmd_msg.twist.angular.z = k_angular * std::atan2(lookahead_y, lookahead_x);
          cmd_msg.header.stamp = this->now();

          // Publish command
          cmd_vel_pub_->publish(cmd_msg);
        }
        else // If the intermediate waypoint is within lookahead, move it closer to the next waypoint
        {
          intermediate_waypoint.position.x += (dx_waypoints / distance_waypoints) * 0.1;
          intermediate_waypoint.position.y += (dy_waypoints / distance_waypoints) * 0.1;
        }

        loop_rate.sleep();
      } while (rclcpp::ok());

      current_waypoint = next_waypoint;


      auto target_pose = goal->path.poses.back().pose;
      double distance_to_goal = std::hypot(
          target_pose.position.x - robot_pose_.position.x,
          target_pose.position.y - robot_pose_.position.y);

      feedback->distance_to_goal = distance_to_goal;
      feedback->current_speed = cmd_msg.twist.linear.x;
      feedback->status = "Waypoint " + std::to_string(i) + "/" + std::to_string(n_wpts) + " reached.";
      goal_handle->publish_feedback(feedback);
    }
    goal_handle->succeed(result);
    cmd_msg.twist.linear.x = 0.0;
    cmd_msg.twist.angular.z = 0.0;
    cmd_msg.header.stamp = this->now();

    cmd_vel_pub_->publish(cmd_msg);

    // feedback->distance_to_goal = distance_to_goal;
    // feedback->current_speed = cmd_msg.twist.linear.x;
    // feedback->status = "Waypoint " + std::to_string(n_wpts) + "/" + std::to_string(n_wpts) + " reached.";
    // goal_handle->publish_feedback(feedback);
  }

  void robot_pose_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
  {
    auto odom_msg_pose_cov = odom_msg->pose;
    robot_pose_ = odom_msg_pose_cov.pose;
    RCLCPP_DEBUG(this->get_logger(), "Updated robot pose: x=%.2f, y=%.2f",
                robot_pose_.position.x,
                robot_pose_.position.y);
  }

  geometry_msgs::msg::Pose transformGlobalToLocal(
      const geometry_msgs::msg::Pose& robot_global_pose,
      const geometry_msgs::msg::Pose& object_global_pose)
  {
      tf2::Transform tf_robot_global, tf_object_global;

      // 1. Convert ROS messages to TF2 objects
      tf2::fromMsg(robot_global_pose, tf_robot_global);
      tf2::fromMsg(object_global_pose, tf_object_global);

      // 2. Calculate the Relative Transform
      // We need the transform FROM robot TO global (inverse of where the robot is)
      // Then we apply that to the object.
      tf2::Transform tf_object_local = tf_robot_global.inverse() * tf_object_global;

      // 3. Convert back to ROS message
      geometry_msgs::msg::Pose object_local_pose;
      tf2::toMsg(tf_object_local, object_local_pose);

      return object_local_pose;
  }

  // Members
  rclcpp_action::Server<FollowPath>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;

  geometry_msgs::msg::Pose robot_pose_;
  double lookahead_distance_; // meters
  double distance_threshold_ = 0.1; // meters
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
