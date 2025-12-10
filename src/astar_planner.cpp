#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <iostream>
#include <queue>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace std;

struct AStarNode
{
    int x, y;
    float f, g, h;
    bool operator>(const AStarNode &other) const
    {
        return f > other.f;
    }
};

class AStarPlanner : public rclcpp::Node
{
public:
    AStarPlanner() : Node("astar_planner")
    {
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
    vector<vector<int>> grid_;
    
    geometry_msgs::msg::PoseStamped start_, goal_;
    bool start_received_ = false, goal_received_ = false;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_ = *msg;
        map_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "Map received");
        tryPlanning();
    }

    void startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        start_ = *msg;
        start_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "start_ received");
        tryPlanning();
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_ = *msg;
        goal_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "goal_ received");

        tryPlanning();
    }

    void tryPlanning()
    {
        if (map_received_ && start_received_ && goal_received_)
        {
            int width = map_.info.width;
            int height = map_.info.height;

            std::vector<std::vector<int>> grid(height, std::vector<int>(width));

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    int index = y * width + x;
                    grid_[y][x] = static_cast<int>(map_.data[index]);
                }
            }

            RCLCPP_INFO(this->get_logger(), "Running A*...");
            runAStar();
        }
    }

    double heuristic_a(const std::pair<int, int> &goal, const std::pair<int, int> &current)
    {
        double h = std::sqrt((std::pow(goal.second - current.second, 2)) + (std::pow(goal.first - current.first, 2)));
        std::cout << "h: " << h << std::endl;
        return h;
    }

    void runAStar()
    {
        priority_queue<
            pair<double, pair<int, int>>,
            vector<pair<double, pair<int, int>>>,
            greater<pair<double, pair<int, int>>>>
            frontier;

        int rows = 30, cols = 30;

        vector<vector<int>> grid(rows, vector<int>(cols, 0)); // 0 = livre, 1 = obstáculo

        int sx = 0, sy = 0; // start
        pair<int, int> start = {sx, sy};
        pair<int, int> goal = {4, 15};

        frontier.push({0, start});

        vector<vector<pair<int, int>>> came_from(rows, vector<pair<int, int>>(cols, {-2, -2}));
        vector<vector<double>> cost_so_far(rows, vector<double>(cols, {std::numeric_limits<double>::max()}));
        came_from[sx][sy] = {-1, -1};
        cost_so_far[sx][sy] = 0;

        int dx[8] = {1, -1, 0, 0, 1, -1, -1, 1};
        int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!frontier.empty())
        {
            auto [x, y] = frontier.top().second;
            frontier.pop();

            if (x == goal.first && y == goal.second)
            {
                break;
            }

            for (int i = 0; i < 8; i++)
            {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (nx < 0 || nx >= rows || ny < 0 || ny >= cols)
                {
                    continue;
                }

                if (grid[nx][ny] == 1)
                {
                    continue;
                }

                double new_cost = cost_so_far[x][y] + 1;
                if (cost_so_far[nx][ny] == std::numeric_limits<double>::max() || new_cost < cost_so_far[nx][ny])
                {
                    cost_so_far[nx][ny] = new_cost;
                    double priority = new_cost + AStarPlanner::heuristic_a(goal, pair<int, int>{nx, ny});
                    frontier.push({priority, {nx, ny}});
                    came_from[nx][ny] = {x, y};
                }
            }
        }

        vector<pair<int, int>> path;
        pair<int, int> cur = goal;

        if (came_from[cur.first][cur.second].first == -2)
        {
            cout << "Nenhum caminho encontrado!\n";
        }
        else
        {
            while (!(cur.first == -1 && cur.second == -1))
            {
                path.push_back(cur);
                cur = came_from[cur.first][cur.second];
            }
            reverse(path.begin(), path.end());

            cout << "Path encontrado:\n";
            for (auto &p : path)
            {
                cout << "(" << p.first << "," << p.second << ")\n";
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}