#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>

struct AStarNode {
    int x, y;
    float f, g, h;
    bool operator>(const AStarNode& other) const {
        return f > other.f;
    }
};

class AStarPlanner : public rclcpp::Node {
public:
    AStarPlanner() : Node("astar_planner") {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1));

        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/start", 10, std::bind(&AStarPlanner::startCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10, std::bind(&AStarPlanner::goalCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    }

private:

    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;

    geometry_msgs::msg::PoseStamped start_, goal_;
    bool start_received_ = false, goal_received_ = false;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_ = *msg;
        map_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "Map received");
        tryPlanning();
    }

    void startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        start_ = *msg;
        start_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "start_ received");
        tryPlanning();
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_ = *msg;
        goal_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "goal_ received");

        tryPlanning();
    }

    void tryPlanning() {
        if (map_received_ && start_received_ && goal_received_) {
            RCLCPP_INFO(this->get_logger(), "Running A*...");
            runAStar();
        }
    }

    bool isFree(int x, int y) {
        int idx = y * map_.info.width + x;
        if (idx < 0 || idx >= (int)map_.data.size()) return false;
        return map_.data[idx] == 0;
    }

    float heuristic(int x1, int y1, int x2, int y2) {
        return std::hypot(x2 - x1, y2 - y1);
    }

    void runAStar() {
        int w = map_.info.width;
        int h = map_.info.height;

        int sx = (start_.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
        int sy = (start_.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;

        int gx = (goal_.pose.position.x - map_.info.origin.position.x) / map_.info.resolution;
        int gy = (goal_.pose.position.y - map_.info.origin.position.y) / map_.info.resolution;

        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;

        std::unordered_map<int, float> g_cost;
        std::unordered_map<int, int> came_from;

        auto index = [&](int x, int y) { return y * w + x; };

        AStarNode start_node {sx, sy, 0, 0, heuristic(sx, sy, gx, gy)};
        start_node.f = start_node.h;

        open.push(start_node); 
        g_cost[index(sx, sy)] = 0.0f;

        std::vector<std::pair<int,int>> neighbors = {
            {1,0},{-1,0},{0,1},{0,-1}
        };

        bool found = false;

        while (!open.empty()) {
            AStarNode current = open.top();
            open.pop();

            if (current.x == gx && current.y == gy) {
                found = true;
                break;
            }

            for (auto& nb : neighbors) {
                int nx = current.x + nb.first;
                int ny = current.y + nb.second;

                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                if (!isFree(nx, ny)) continue;

                float new_g = g_cost[index(current.x, current.y)] + 1.0;

                if (!g_cost.count(index(nx, ny)) || new_g < g_cost[index(nx, ny)]) {
                    g_cost[index(nx, ny)] = new_g;

                    float h = heuristic(nx, ny, gx, gy);
                    AStarNode next {nx, ny, new_g + h, new_g, h};
                    open.push(next);

                    came_from[index(nx, ny)] = index(current.x, current.y);
                }
            }
        }

        if (!found) {
            RCLCPP_WARN(this->get_logger(), "A* failed: no path.");
            return;
        }

        // Reconstruct path
        nav_msgs::msg::Path path;
        path.header.frame_id = map_.header.frame_id;
        path.header.stamp = now();

        int cx = gx, cy = gy;
        while (!(cx == sx && cy == sy)) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = cx * map_.info.resolution + map_.info.origin.position.x;
            pose.pose.position.y = cy * map_.info.resolution + map_.info.origin.position.y;
            path.poses.push_back(pose);

            int idx = index(cx, cy);
            auto it = came_from.find(idx);
            if (it == came_from.end()) break;

            int prev = it->second;
            cx = prev % w;
            cy = prev / w;
        }

        std::reverse(path.poses.begin(), path.poses.end());
        path_pub_->publish(path);

        RCLCPP_INFO(this->get_logger(), "A* path published.");
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}