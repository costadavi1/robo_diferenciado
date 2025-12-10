#include <SFML/Graphics.hpp>
#include <iostream>
#include <queue>
#include <vector>
using namespace std;

int main() {
    const int cellSize = 40;
    const int rows = 10;
    const int cols = 15;

    sf::RenderWindow window(sf::VideoMode(cols * cellSize, rows * cellSize), "Visualizador A* - Exemplo");

    sf::RectangleShape cell(sf::Vector2f(cellSize - 1, cellSize - 1));

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Black);

        // desenha grid
        for (int y = 0; y < rows; y++) {
            for (int x = 0; x < cols; x++) {
                cell.setPosition(x * cellSize, y * cellSize);
                cell.setFillColor(sf::Color(70, 70, 70));
                window.draw(cell);
            }
        }

        window.display();
    }

    return 0;
}