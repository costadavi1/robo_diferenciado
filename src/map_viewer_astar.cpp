#include <SFML/Graphics.hpp>
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

double heuristic(const std::pair<int, int> &goal, const std::pair<int, int> &current)
{
    double h = std::sqrt((std::pow(goal.second - current.second, 2)) + (std::pow(goal.first - current.first, 2)));
    std::cout << "h: " << h << std::endl;
    return h;
}

int main()
{

    using Node = pair<int, int>; // Exemplo de nó (pode ser int se quiser)

    priority_queue<
        pair<double, Node>,
        vector<pair<double, Node>>,
        greater<pair<double, Node>>>
        frontier;

    int rows = 30, cols = 30;

    vector<vector<int>> grid(rows, vector<int>(cols, 0)); // 0 = livre, 1 = obstáculo

    int sx = 0, sy = 0; // start

    // pair<int, int> goal;
    // goal = {8,8};
    // queue<pair<int, int>> frontier;
    Node start = {sx, sy};
    Node goal = {4, 15};

    frontier.push({0, start});

    // came_from[x][y] = pai da célula (x,y)
    vector<vector<pair<int, int>>> came_from(rows, vector<pair<int, int>>(cols, {-2, -2}));
    vector<vector<double>> cost_so_far(rows, vector<double>(cols, {std::numeric_limits<double>::max()}));
    came_from[sx][sy] = {-1, -1}; // start não tem pai
    cost_so_far[sx][sy] = 0;      // start não custo

    // Quatro direções
    int dx[8] = {1, -1, 0, 0, 1, -1, -1, 1};
    int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    const int cellSize = 40;

    sf::RenderWindow window(sf::VideoMode(cols * cellSize, rows * cellSize), "Visualizador A* - Exemplo");

    sf::RectangleShape cell(sf::Vector2f(cellSize - 1, cellSize - 1));

    sf::Font font;
    if (!font.loadFromFile("/home/davi/Downloads/arial/ARIAL.TTF"))
    {
        printf("ERRO: Não encontrou arial.ttf\n");
        return 1;
    }
    sf::Text text;
    text.setFont(font);
    text.setCharacterSize(16);
    text.setFillColor(sf::Color::White);

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

            // já visitado?
            // if (came_from[nx][ny].first != -2)
            // {
            // std::cout << "oba4" << std::endl;
            // continue;
            // }

            if (grid[nx][ny] == 1)
            {
                continue;
            }

            double new_cost = cost_so_far[x][y] + 1;
            if (cost_so_far[nx][ny] == std::numeric_limits<double>::max() || new_cost < cost_so_far[nx][ny])
            {
                cost_so_far[nx][ny] = new_cost;
                double priority = new_cost + heuristic(goal, {nx, ny});
                frontier.push({priority, {nx, ny}});
                came_from[nx][ny] = {x, y};
                std::cout << "added- x:" << nx << ", y:" << ny << ",cost: " << new_cost << std::endl;
               
                // auto [px, py] = came_from[nx][ny];
                cell.setPosition(x * cellSize, y * cellSize);
                cell.setFillColor(sf::Color(70, 70, 70));
                window.draw(cell);
                window.display();
                this_thread::sleep_for(chrono::milliseconds(250));


            }
        }
    }

    vector<pair<int,int>> path;

    pair<int,int> cur = goal;

    // Se goal não foi alcançado, ele ainda tem pai = (-2,-2)
    if (came_from[cur.first][cur.second].first == -2) {
        cout << "Nenhum caminho encontrado!\n";
    } else {
        while (!(cur.first == -1 && cur.second == -1)) {
            path.push_back(cur);
            cur = came_from[cur.first][cur.second];
        }

        // O vetor está invertido (do goal para start)
        reverse(path.begin(), path.end());

        cout << "Path encontrado:\n";
        for (auto &p : path) {
            cout << "(" << p.first << "," << p.second << ")\n";
        }
    }
    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Black);


    // window.clear();
    // for (int i = 0; i < rows; i++)
    // {
    //     for (int j = 0; j < cols; j++)
    //     {
    //         auto [px, py] = came_from[i][j];
    //         cell.setPosition(px * cellSize, py * cellSize);
    //         cell.setFillColor(sf::Color(70, 70, 70));
    //         window.draw(cell);

    //         if (cost_so_far[i][j] != std::numeric_limits<double>::max())
    //         {
    //         std::ostringstream oss;
    //         oss << std::fixed << std::setprecision(0) << cost_so_far[i][j];
    //         std::string s = oss.str();
    //         text.setString(s);

    //         sf::FloatRect bounds = text.getLocalBounds();
    //         float cx = py * cellSize + cellSize / 2.f - bounds.width / 2.f;
    //         float cy = px * cellSize + cellSize / 2.f - bounds.height;

    //         text.setPosition(cx, cy);

    //         window.draw(text);
    //         }
    //     }
    // }

    // window.display();
    // this_thread::sleep_for(chrono::milliseconds(250));

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
    }

    return 0;
}