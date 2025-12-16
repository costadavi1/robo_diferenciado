#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose.hpp>

#include <queue>
#include <cmath>
#include <unordered_map>
#include <queue>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace std;

struct PGMImage
{
    string magic_number;
    int width, height, max_val;
    vector<unsigned char> pixels;
};

bool readPGM(const string &filename, PGMImage &img)
{
    ifstream infile(filename, ios::binary); // Open in binary mode for both types
    if (!infile)
    {
        cerr << "Error: Cannot open file " << filename << " for reading" << endl;
        return false;
    }

    string line;
    getline(infile, line);
    stringstream ss(line);
    ss >> img.magic_number;

    if (img.magic_number != "P2" && img.magic_number != "P5")
    {
        cerr << "Error: Not a valid PGM file format" << endl;
        return false;
    }

    while (getline(infile, line))
    {
        if (line[0] != '#')
        {
            break;
        }
    }

    stringstream ss_data(line);
    ss_data >> img.width >> img.height;

    if (ss_data.fail())
    {
        getline(infile, line);
        stringstream ss_height(line);
        ss_height >> img.height;
    }

    infile >> img.max_val;

    infile.ignore(1);

    img.pixels.resize(img.width * img.height);

    if (img.magic_number == "P2")
    {
        int pixel_val;
        for (int i = 0; i < img.width * img.height; ++i)
        {
            infile >> pixel_val;
            img.pixels[i] = static_cast<unsigned char>(pixel_val);
        }
    }
    else if (img.magic_number == "P5")
    {
        infile.read(reinterpret_cast<char *>(img.pixels.data()), img.pixels.size());
    }

    infile.close();
    return true;
}

struct MapMetaData
{
    std::string image;
    double resolution;
    std::vector<double> origin;
    int negate;
    double occupied_thresh;
    double free_thresh;
};

std::vector<int8_t> ConvertPGM(
    const PGMImage &image,
    int negate,
    double occupied_thresh,
    double free_thresh)
{
    // std::vector<uint8_t> img(image.width * image.height);
    std::vector<int8_t> data(image.width * image.height);
    for (int y = 0; y < image.height; y++)
    {
        for (int x = 0; x < image.width; x++)
        {

            int i = x + (image.height - y - 1) * image.width;
            uint8_t pixel = image.pixels[y * image.width + x];

            if (negate)
            {
                pixel = 255 - pixel;
            }

            double prob = (255.0 - pixel) / 255.0;

            if (prob > occupied_thresh)
            {
                data[i] = 100;
            }
            else if (prob < free_thresh)
            {
                data[i] = 0;
            }
            else
            {
                data[i] = -1;
            }
        }
    }
    return data;
}



MapMetaData loadYaml(const std::string &yaml_file)
{
    YAML::Node doc = YAML::LoadFile(yaml_file);

    MapMetaData map;
    map.image = doc["image"].as<std::string>();
    map.resolution = doc["resolution"].as<double>();
    map.origin = doc["origin"].as<std::vector<double>>();
    map.negate = doc["negate"].as<int>();
    map.occupied_thresh = doc["occupied_thresh"].as<double>();
    map.free_thresh = doc["free_thresh"].as<double>();

    return map;
}

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
        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/start", 10, std::bind(&AStarPlanner::startCallback, this, std::placeholders::_1));

        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&AStarPlanner::odomCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal", 10, std::bind(&AStarPlanner::goalCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

        map_received_ = importMap("/home/davi/project_ws/src/robo_diferenciado/map/map.yaml", "/home/davi/project_ws/src/robo_diferenciado/map/map.pgm");
    }

private:
    nav_msgs::msg::OccupancyGrid map_;
    bool map_received_ = false;
    vector<vector<int>> grid_;

    geometry_msgs::msg::PoseStamped start_, goal_;
    bool start_received_ = false, goal_received_ = false;
    std::pair<int, int> goal_point_;
    std::pair<int, int> start_point_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::OccupancyGrid createOccupancyGrid(
    const MapMetaData &map,
    const std::vector<int8_t> &data,
    int width,
    int height)
{
    nav_msgs::msg::OccupancyGrid grid;

    if (!data.empty())
    {
        grid.header.stamp = this->get_clock()->now();
        grid.header.frame_id= "map";
        grid.info.resolution = map.resolution;
        grid.info.width = width;
        grid.info.height = height;

        grid.info.origin.position.x = map.origin[0];
        grid.info.origin.position.y = map.origin[1];
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;

        grid.data = data;
    }
    else
    {
        std::cout << "empty" << endl;
    }

    return grid;
}

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double wx = msg->pose.pose.position.x + 0.5;
        double wy = msg->pose.pose.position.y + 0.5;

        start_point_ = {
            static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution),
            static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution)};

        start_received_ = true;
        map_pub_->publish(map_);

        tryPlanning();
    }

    bool importMap(const std::string &yaml_path, const std::string &pgm_path)
    {
        MapMetaData meta_data;
        meta_data = loadYaml(yaml_path);
        cout << meta_data.occupied_thresh << endl;

        PGMImage image;
        if (readPGM(pgm_path, image))
        {
            std::vector<int8_t> data = ConvertPGM(image, meta_data.negate, meta_data.occupied_thresh, meta_data.free_thresh);
            map_ = createOccupancyGrid(meta_data, data, image.width, image.height);

            return true;
        }
        else
        {
            cout << "Failed to read image." << endl;
            return false;
        }
    }

    void startCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        start_ = *msg;
        RCLCPP_INFO_ONCE(this->get_logger(), "start_ received");

        tryPlanning();
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // goal_ = *msg;
        // goal_point_ = {msg->pose.position.x, msg->pose.position.y};

        double wx = msg->pose.position.x;
        double wy = msg->pose.position.y;
        goal_point_ = {
            static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution),
            static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution)};
        goal_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "goal_ received");

        RCLCPP_INFO_ONCE(this->get_logger(), "world goal x: %f", wx);
        RCLCPP_INFO_ONCE(this->get_logger(), "world goal y: %f", wy);

        RCLCPP_INFO_ONCE(this->get_logger(), "index goal x: %i", goal_point_.first);
        RCLCPP_INFO_ONCE(this->get_logger(), "index goal y: %i", goal_point_.second);

        tryPlanning();
    }

    void tryPlanning()
    {
        if (map_received_ && start_received_ && goal_received_)
        {
            int width = map_.info.width;
            int height = map_.info.height;

            std::vector<std::vector<int>> grid(height, std::vector<int>(width));

            grid_ = grid;

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
        // std::cout << "h: " << h << std::endl;
        return h;
    }

    void runAStar()
    {
        std::cout << "Aqui1 " << std::endl;

        priority_queue<
            pair<double, pair<int, int>>,
            vector<pair<double, pair<int, int>>>,
            greater<pair<double, pair<int, int>>>>
            frontier;

        pair<int, int> start = start_point_;
        pair<int, int> goal = goal_point_;

        frontier.push({0, start});

        vector<vector<pair<int, int>>> came_from(map_.info.height, vector<pair<int, int>>(map_.info.width, {-2, -2}));
        vector<vector<double>> cost_so_far(map_.info.height, vector<double>(map_.info.width, std::numeric_limits<double>::max()));
        came_from[start_point_.first][start_point_.second] = {-1, -1};
        cost_so_far[start_point_.first][start_point_.second] = 0;


        int dx[8] = {1, -1, 0, 0, 1, -1, -1, 1};
        int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!frontier.empty())
        {
            auto [x, y] = frontier.top().second;
            frontier.pop();

            if (x == goal.first && y == goal.second)
            {
                std::cout << "path encontrado" << std::endl;
                break;
            }

            for (int i = 0; i < 8; i++)
            {
                int nx = x + dx[i];
                int ny = y + dy[i];

                if (nx < 0 || nx >= map_.info.height || ny < 0 || ny >= map_.info.width)
                {
                    continue;
                }

                if (grid_[ny][nx] > 65)
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

        nav_msgs::msg::Path path_out;

        path_out.header.frame_id = "map";
        path_out.header.stamp = this->get_clock()->now();

        for (const auto &cell : path)
        {
            int x = cell.first;
            int y = cell.second;

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_out.header;

            pose.pose.position.x =
                map_.info.origin.position.x +
                (x + 0.5) * map_.info.resolution;

            pose.pose.position.y =
                map_.info.origin.position.y +
                (y + 0.5) * map_.info.resolution;

            pose.pose.position.z = 0.0;

            pose.pose.orientation.w = 1.0;

            path_out.poses.push_back(pose);
        }

        path_pub_->publish(path_out);

        start_received_ = false;
        goal_received_ = false;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AStarPlanner>());
    rclcpp::shutdown();
    return 0;
}