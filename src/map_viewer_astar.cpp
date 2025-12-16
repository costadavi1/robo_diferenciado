#include <SFML/Graphics.hpp>
#include <iostream>
#include <queue>
#include <vector>
#include <iostream>
#include <chrono>
#include <thread>

using namespace std;

int main()
{

    using Node = pair<int, int>; // Exemplo de nó (pode ser int se quiser)

    priority_queue<
        pair<int, Node>,
        vector<pair<int, Node>>,
        greater<pair<int, Node>>>
        frontier;

    int rows = 10, cols = 15;

    vector<vector<int>> grid(rows, vector<int>(cols, 0)); // 0 = livre, 1 = obstáculo

    int sx = 0, sy = 0; // start

    // pair<int, int> goal;
    // goal = {8,8};
    // queue<pair<int, int>> frontier;
    Node start = {1, 1};
    Node goal = {8, 8};

    frontier.push({0, start});

    // came_from[x][y] = pai da célula (x,y)
    vector<vector<pair<int, int>>> came_from(rows, vector<pair<int, int>>(cols, {-2, -2}));
    vector<vector<int>> cost_so_far(rows, vector<int>(cols, {-2}));
    came_from[sx][sy] = {-1, -1}; // start não tem pai
    cost_so_far[sx][sy] = 0;      // start não custo

    // Quatro direções
    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    const int cellSize = 40;

    sf::RenderWindow window(sf::VideoMode(cols * cellSize, rows * cellSize), "Visualizador A* - Exemplo");

    sf::RectangleShape cell(sf::Vector2f(cellSize - 1, cellSize - 1));

    // while (window.isOpen())
    // {
    //     sf::Event event;
    //     while (window.pollEvent(event))
    //     {
    //         if (event.type == sf::Event::Closed)
    //             window.close();
    //     }

    //     window.clear(sf::Color::Black);

        while (!frontier.empty())
        {
            auto [x, y] = frontier.top().second;
            frontier.pop();
            std::cout << "ponto: x-" << x<< ", y- "<< y << std::endl;

            for (int i = 0; i < 4; i++)
            {
                std::cout << "oba2" << std::endl;

                int nx = x + dx[i];
                int ny = y + dy[i];

                // dentro da grid
                if (nx < 0 || nx >= rows || ny < 0 || ny >= cols)
                {
                    std::cout << "oba3" << std::endl;
                    continue;
                }

                // já visitado?
                if (came_from[nx][ny].first != -2)
                {
                    std::cout << "oba4" << std::endl;
                    continue;
                }

                // obstáculo?
                if (grid[nx][ny] == 1)
                {
                    std::cout << "oba5" << std::endl;
                    continue;
                }

                // if(nx == goal.first && ny == goal.second)
                // {
                //   break;
                // }

                std::cout << "oba6" << std::endl;

                double new_cost = cost_so_far[x][y] + 1;

                std::cout << "new_cost" << new_cost << std::endl;
                std::cout << "cost_so_far[x][y]" << cost_so_far[x][y] << std::endl;
                std::cout << "cost_so_far[nx][ny]" <<cost_so_far[nx][ny] << std::endl;

                if (cost_so_far[nx][ny] != -2 || new_cost < cost_so_far[nx][ny])
                {
                    // std::cout << "oba7" << std::endl;

                    cost_so_far[nx][ny] = new_cost;
                    frontier.push({new_cost, {nx, ny}});
                    came_from[nx][ny] = {x, y};
                    std::cout << "added- x:" << nx << ", y:" << ny << ",cost: " << new_cost << std::endl;
                }

                // window.clear(sf::Color::Black);

                // for (int i = 0; i < rows; i++)
                // {
                //     for (int j = 0; j < cols; j++)
                //     {
                //         auto [px, py] = came_from[i][j];
                //         cell.setPosition(px * cellSize, py * cellSize);
                //         cell.setFillColor(sf::Color(70, 70, 70));
                //         window.draw(cell);
                //     }
                // }
                // window.display();
                this_thread::sleep_for(chrono::milliseconds(250));
            }
        }

        // Exemplo: imprimir came_from
        // cout << "Pais das células visitadas:\n";
        // for (int i = 0; i < rows; i++) {
        //     for (int j = 0; j < cols; j++) {
        //         auto [px, py] = came_from[i][j];
        //         cout << "(" << px << "," << py << ") ";
        //     }
        //     cout << "\n";
        // }

        // // desenha grid
        // for (int y = 0; y < rows; y++) {
        //     for (int x = 0; x < cols; x++) {
        //         cell.setPosition(x * cellSize, y * cellSize);
        //         cell.setFillColor(sf::Color(70, 70, 70));
        //         window.draw(cell);
        //     }
        // }

        // window.display();
    // }

    return 0;
}