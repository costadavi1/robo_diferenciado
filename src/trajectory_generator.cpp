// smooth_trajectory_path_publisher.cpp
// Small ROS2 node that publishes a hardcoded set of poses as nav_msgs::msg::Path
// Orientations are set to identity (no yaw used)
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <Eigen/Dense>
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
        this->path_pub_ = this->create_publisher<nav_msgs::msg::Path>("generated_path", 10);

        this->timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TrajectoryGenerator::send_goal, this));
    }
private:
    /**
     * @brief Get the knot t object
     *
     * @param t
     * @param alpha
     * @param p0
     * @param p1
     * @return float
     */
    float get_knot_t(float t,
                    float alpha,
                    const geometry_msgs::msg::PoseStamped& p0,
                    const geometry_msgs::msg::PoseStamped& p1 )
    {
        float dx  = p1.pose.position.x - p0.pose.position.x;
        float dy  = p1.pose.position.y - p0.pose.position.y;
        float arc = std::sqrt(dx*dx + dy*dy);

        float knot = std::pow(arc, alpha);

        return (knot + t);
    }

    std::vector<geometry_msgs::msg::PoseStamped> generateSpline(
      const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
      int points_per_segment = 10)
    {
        std::vector<geometry_msgs::msg::PoseStamped> smooth_path;
        std::vector<geometry_msgs::msg::PoseStamped> ctrl_points = waypoints;

        // Catmull-Rom requires 4 points: P0 (prev), P1 (start), P2 (end), P3 (next)
        // To handle start/end, add extrapolated points at beginning and end
        // they cannnot be the same as the first/last point, or the division by zero will occur
        if (waypoints.size() >= 2) {
            auto p0 = waypoints[0];
            auto p1 = waypoints[1];
            auto p_last = waypoints.back(); // (2, 2)
            auto p_prev = waypoints[waypoints.size() - 2]; // (2, 1)

            geometry_msgs::msg::PoseStamped interpolated_point = p0;
            interpolated_point.pose.position.x = p0.pose.position.x - 0.1*(p1.pose.position.x - p0.pose.position.x);
            interpolated_point.pose.position.y = p0.pose.position.y - 0.1*(p1.pose.position.y - p0.pose.position.y);

            geometry_msgs::msg::PoseStamped extrapolated_point = p_last;
            extrapolated_point.pose.position.x = p_last.pose.position.x + 0.1*(p_last.pose.position.x - p_prev.pose.position.x);
            extrapolated_point.pose.position.y = p_last.pose.position.y + 0.1*(p_last.pose.position.y - p_prev.pose.position.y);

            ctrl_points.insert(ctrl_points.begin(), interpolated_point);
            ctrl_points.push_back(extrapolated_point);
        }
        else
        {
            return waypoints; // Not enough points to create a spline
        }

        for (size_t i = 0; i < ctrl_points.size(); ++i)
        {
            RCLCPP_INFO(this->get_logger(),
                "Control Points: (%.2f, %.2f)",
                ctrl_points[i].pose.position.x,
                ctrl_points[i].pose.position.y
            );
        }


        for (size_t i = 1; i < ctrl_points.size() - 2; ++i) {
            for (int j = 0; j < points_per_segment; ++j) {
            float t = (float)j / (float)points_per_segment;
            auto new_point = calculateCatmullRom(
                    ctrl_points[i-1], ctrl_points[i],
                    ctrl_points[i+1], ctrl_points[i+2], t);
            geometry_msgs::msg::PoseStamped n_pose_stamped;
            n_pose_stamped.pose.position.x = new_point.x();
            n_pose_stamped.pose.position.y = new_point.y();


            smooth_path.push_back(n_pose_stamped);
            }


        }
        // Add the very last point
        smooth_path.push_back(waypoints.back());
        return smooth_path;
    }

    /**
     * @brief Calculate the Catmull-Rom spline point
     *
     * @param p0
     * @param p1
     * @param p2
     * @param p3
     * @param t_normalized
     * @param alpha
     * @return Eigen::Vector2d
     */
    Eigen::Vector2d calculateCatmullRom(
        const geometry_msgs::msg::PoseStamped& p0,
        const geometry_msgs::msg::PoseStamped& p1,
        const geometry_msgs::msg::PoseStamped& p2,
        const geometry_msgs::msg::PoseStamped& p3,
        float t_normalized,
        float alpha = 0.5f)
    {
        Eigen::Vector2d p0_vec(p0.pose.position.x, p0.pose.position.y);
        Eigen::Vector2d p1_vec(p1.pose.position.x, p1.pose.position.y);
        Eigen::Vector2d p2_vec(p2.pose.position.x, p2.pose.position.y);
        Eigen::Vector2d p3_vec(p3.pose.position.x, p3.pose.position.y);

        float t0 = 0.0f;
        float t1 = get_knot_t( t0, alpha, p0, p1 );
        float t2 = get_knot_t( t1, alpha, p1, p2 );
        float t3 = get_knot_t( t2, alpha, p2, p3 );

        // Linear interpolation function
        auto lerp = [](float a, float b, float t) {
            return a + t * (b - a);
        };
        float t = lerp( t1, t2, t_normalized );

        Eigen::Vector2d A1 = ( t1-t )/( t1-t0 )*p0_vec + ( t-t0 )/( t1-t0 )*p1_vec;
        Eigen::Vector2d A2 = ( t2-t )/( t2-t1 )*p1_vec + ( t-t1 )/( t2-t1 )*p2_vec;
        Eigen::Vector2d A3 = ( t3-t )/( t3-t2 )*p2_vec + ( t-t2 )/( t3-t2 )*p3_vec;
        Eigen::Vector2d B1 = ( t2-t )/( t2-t0 )*A1 + ( t-t0 )/( t2-t0 )*A2;
        Eigen::Vector2d B2 = ( t3-t )/( t3-t1 )*A2 + ( t-t1 )/( t3-t1 )*A3;
        Eigen::Vector2d C  = ( t2-t )/( t2-t1 )*B1 + ( t-t1 )/( t2-t1 )*B2;

        return C;
    }


    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel(); // Only send once

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return;
        }

        auto goal_msg = FollowPath::Goal();
        auto smooth_path_msg = FollowPath::Goal();

        // --- CONSTRUCT A DUMMY PATH ---
        goal_msg.path.header.stamp = this->now();
        goal_msg.path.header.frame_id = "odom";
        smooth_path_msg = goal_msg;

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

        auto smooth_path = generateSpline(goal_msg.path.poses, 20);

        for (const auto& pose : smooth_path) {
            RCLCPP_INFO(this->get_logger(),
                "Smooth Path Point: (%.2f, %.2f)",
                pose.pose.position.x,
                pose.pose.position.y
            );
            smooth_path_msg.path.poses.push_back(pose);
        }

        path_pub_->publish(smooth_path_msg.path);

        RCLCPP_INFO(this->get_logger(), "Sending path goal...");

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

        // Define the 3 callbacks: Response, Feedback, Result
        send_goal_options.goal_response_callback =
        std::bind(&TrajectoryGenerator::goal_response_callback, this, _1);

        send_goal_options.feedback_callback =
        std::bind(&TrajectoryGenerator::feedback_callback, this, _1, _2);

        send_goal_options.result_callback =
        std::bind(&TrajectoryGenerator::result_callback, this, _1);

        this->client_ptr_->async_send_goal(smooth_path_msg, send_goal_options);
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
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
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
