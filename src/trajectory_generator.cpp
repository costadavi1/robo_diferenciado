// smooth_trajectory_path_publisher.cpp
// Small ROS2 node that publishes a hardcoded set of poses as nav_msgs::msg::Path
// Orientations are set to identity (no yaw used)
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "robo_diferenciado/action/follow_path.hpp"


using namespace std::chrono_literals;

class TrajectoryGenerator : public rclcpp::Node
{
public:
    using FollowPath = robo_diferenciado::action::FollowPath;
    using GoalHandlePathFollower = rclcpp_action::ClientGoalHandle<FollowPath>;

    TrajectoryGenerator()
        : Node("trajectory_generator_node")
    {

        this->client_ptr_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TrajectoryGenerator::send_goal, this));
    }
private:
    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel(); // Only send once

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
        }

        auto goal_msg = FollowPath::Goal();

        // --- CONSTRUCT A DUMMY PATH ---
        goal_msg.path.header.stamp = this->now();
        goal_msg.path.header.frame_id = "odom";

        // Point 1: (0,0)
        auto pose1 = geometry_msgs::msg::PoseStamped();
        pose1.pose.position.x = 0.0;
        pose1.pose.position.y = 0.0;
        goal_msg.path.poses.push_back(pose1);

        // Point 2: (1,0) - Forward
        auto pose2 = geometry_msgs::msg::PoseStamped();
        pose2.pose.position.x = 1.0;
        pose2.pose.position.y = 0.0;
        goal_msg.path.poses.push_back(pose2);

        // Point 3: (1,1) - Left
        auto pose3 = geometry_msgs::msg::PoseStamped();
        pose3.pose.position.x = 1.0;
        pose3.pose.position.y = 1.0;
        goal_msg.path.poses.push_back(pose3);

        auto pose4 = geometry_msgs::msg::PoseStamped();
        pose4.pose.position.x = 2.0;
        pose4.pose.position.y = 1.0;
        goal_msg.path.poses.push_back(pose4);

        auto pose5 = geometry_msgs::msg::PoseStamped();
        pose5.pose.position.x = 2.0;
        pose5.pose.position.y = 2.0;
        goal_msg.path.poses.push_back(pose5);

        RCLCPP_INFO(this->get_logger(), "Sending path goal...");

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

        // Define the 3 callbacks: Response, Feedback, Result
        send_goal_options.goal_response_callback =
        std::bind(&TrajectoryGenerator::goal_response_callback, this, _1);

        send_goal_options.feedback_callback =
        std::bind(&TrajectoryGenerator::feedback_callback, this, _1, _2);

        send_goal_options.result_callback =
        std::bind(&TrajectoryGenerator::result_callback, this, _1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    // Callback 1: Server accepted or rejected the goal?
    void goal_response_callback(const GoalHandlePathFollower::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    // Callback 2: Continuous updates from server
    void feedback_callback(
        GoalHandlePathFollower::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
        "Feedback: Distance Remaining: %.2f m | Speed: %.2f m/s | Status: %s",
        feedback->distance_to_goal,
        feedback->current_speed,
        feedback->status.c_str()
        );
    }

    // Callback 3: Final result when task finishes
    void result_callback(const GoalHandlePathFollower::WrappedResult & result)
    {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Result: SUCCEEDED. Message: %s", result.result->message.c_str());
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Result: ABORTED");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Result: CANCELED");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Result: UNKNOWN CODE");
            break;
        }
        rclcpp::shutdown();
    }

    rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGenerator>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
