#include <robo_diferenciado/AStar.hpp>

using namespace std;


int main()
{
    class AStar
    {
    public:
        bool import_map(const nav_msgs::msg::OccupancyGrid::SharedPtr &map)
        {
            int width = map->info.width;
            int height = map->info.height;

            std::vector<std::vector<int>> grid(height, std::vector<int>(width));

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    int index = y * width + x;
                    grid_[y][x] = static_cast<int>(map->data[index]);
                }
            }
        }

        bool plan_path(const pair<double, double> &start, const pair<double, double> &goal, vector<pair<double, double>> &result)
        {
            if (grid_.empty())
            {
                std::cout << "No map imported. Is not possible to calculate the path" << std::endl;
                return false;
            }

            priority_queue<
                pair<double, pair<int, int>>,
                vector<pair<double, pair<int, int>>>,
                greater<pair<double, pair<int, int>>>>
                frontier;

            frontier.push({0, start});

            vector<vector<pair<int, int>>> came_from(rows_, vector<pair<int, int>>(cols_, {-2, -2}));
            vector<vector<double>> cost_so_far(rows_, vector<double>(cols_, {std::numeric_limits<double>::max()}));
            came_from[start.first][start.second] = {-1, -1};
            cost_so_far[start.first][start.second] = 0;

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

                for (int i = 0; i < 8; ++i)
                {
                    int nx = x + dx[i];
                    int ny = y + dy[i];

                    if (nx < 0 || nx >= rows_ || ny < 0 || ny >= cols_)
                    {
                        continue;
                    }

                    if (grid_[nx][ny] == 100)
                    {
                        continue;
                    }

                    double new_cost = cost_so_far[x][y] + euclidian_distance({x, y}, {nx, ny});
                    if (cost_so_far[nx][ny] == std::numeric_limits<double>::max() || new_cost < cost_so_far[nx][ny])
                    {
                        cost_so_far[nx][ny] = new_cost;
                        double priority = new_cost + euclidian_distance(goal, {nx, ny}); // heuristic
                        frontier.push({priority, {nx, ny}});
                        came_from[nx][ny] = {x, y};
                    }
                }
            }
            // vector<pair<double, double>> path;

            pair<double, double> cur = goal;

            if (came_from[cur.first][cur.second].first == -2)
            {
                cout << "Nenhum caminho encontrado!\n";
            }
            else
            {
                while (!(cur.first == -1 && cur.second == -1))
                {
                    result.push_back(cur);
                    cur = came_from[cur.first][cur.second];
                }

                reverse(result.begin(), result.end());

                cout << "Path encontrado:\n";
                for (auto &p : path)
                {
                    cout << "(" << p.first << "," << p.second << ")\n";
                }
            }
        }

    private:
        double euclidian_distance(const std::pair<int, int> &goal, const std::pair<int, int> &current)
        {
            double h = std::sqrt((std::pow(goal.second - current.second, 2)) + (std::pow(goal.first - current.first, 2)));
            return h;
        }

        int rows_;
        int cols_;
        vector<vector<int>> grid_;
    };
}

#endif // ROBO_DIFERENCIADO_A_STAR_HPP
