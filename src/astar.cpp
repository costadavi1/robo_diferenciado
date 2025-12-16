#include <iostream>
#include <queue>
#include <vector>
using namespace std;

int main() {
    int rows = 5, cols = 5;

    vector<vector<int>> grid(rows, vector<int>(cols, 0)); // 0 = livre, 1 = obstáculo
    
    int sx = 0, sy = 0;  // start

    queue<pair<int,int>> frontier;
    frontier.push({sx, sy});

    // came_from[x][y] = pai da célula (x,y)
    vector<vector<pair<int,int>>> came_from(rows, vector<pair<int,int>>(cols, {-2, -2}));
    came_from[sx][sy] = {-1, -1}; // start não tem pai

    // Quatro direções
    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    while (!frontier.empty()) {
        auto [x, y] = frontier.front();
        frontier.pop();

        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            // dentro da grid
            if (nx < 0 || nx >= rows || ny < 0 || ny >= cols)
                continue;

            // já visitado?
            if (came_from[nx][ny].first != -2)
                continue;

            // obstáculo?
            if (grid[nx][ny] == 1)
                continue;

            frontier.push({nx, ny});
            came_from[nx][ny] = {x, y};
        }
    }

    // Exemplo: imprimir came_from
    cout << "Pais das células visitadas:\n";
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            auto [px, py] = came_from[i][j];
            cout << "(" << px << "," << py << ") ";
        }
        cout << "\n";
    }

    return 0;
}