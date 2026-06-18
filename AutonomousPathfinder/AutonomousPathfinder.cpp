// AutonomousPathfinder.cpp : This file contains the 'main' function. Program execution begins and ends there.
// Name: Dhruva Iyer
// Date: 6/18/2026
// A real-time, graphical simulation built in C++ and SFML that visualizes the A* search algorithm autonomously solving complex 2D mazes.

#include <iostream>

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

const int EMPTY = 0;
const int WALL = 1;
const int START = 2;
const int TARGET = 3;

struct Node {
    int x, y, gCost, hCost;
    Node* parent;

    Node(int xPos, int yPos) {
        x = xPos; y = yPos;
        gCost = 0; hCost = 0;
        parent = nullptr;
    }
    int fCost() const { return gCost + hCost; }
};

int getDistance(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

int main()
{
    std::vector<std::vector<int>> grid = {
        {START, EMPTY, EMPTY, WALL,  EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, WALL,  EMPTY, EMPTY, EMPTY, EMPTY},
        {WALL,  WALL,  EMPTY, WALL,  EMPTY, WALL,  WALL,  WALL,  WALL,  EMPTY, WALL,  EMPTY, WALL,  WALL,  EMPTY},
        {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, WALL,  EMPTY, EMPTY, EMPTY, EMPTY, WALL,  EMPTY},
        {EMPTY, WALL,  WALL,  WALL,  WALL,  WALL,  WALL,  EMPTY, WALL,  WALL,  WALL,  WALL,  EMPTY, WALL,  EMPTY},
        {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, WALL,  EMPTY, EMPTY, EMPTY, EMPTY, WALL,  EMPTY, WALL,  EMPTY},
        {WALL,  WALL,  WALL,  WALL,  WALL,  EMPTY, WALL,  WALL,  WALL,  WALL,  EMPTY, WALL,  EMPTY, WALL,  EMPTY},
        {EMPTY, EMPTY, EMPTY, EMPTY, WALL,  EMPTY, EMPTY, EMPTY, EMPTY, WALL,  EMPTY, WALL,  EMPTY, EMPTY, EMPTY},
        {EMPTY, WALL,  WALL,  EMPTY, WALL,  WALL,  WALL,  WALL,  EMPTY, WALL,  EMPTY, WALL,  WALL,  WALL,  WALL },
        {EMPTY, WALL,  EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, WALL,  EMPTY, EMPTY, EMPTY, EMPTY, EMPTY},
        {EMPTY, WALL,  WALL,  WALL,  WALL,  WALL,  WALL,  WALL,  WALL,  WALL,  WALL,  WALL,  WALL,  WALL,  TARGET}
    };

    float cellSize = 50.0f;
    unsigned int windowWidth = static_cast<unsigned int>(grid[0].size() * cellSize);
    unsigned int windowHeight = static_cast<unsigned int>(grid.size() * cellSize);

    sf::RenderWindow window(sf::VideoMode({ windowWidth, windowHeight }), "A* Real-Time Visualization");

    // Cap the framerate to 30 FPS so we can actually watch the AI "think"
    window.setFramerateLimit(30);

    // AI Variables moved into main for real-time processing frame-by-frame
    int startX = 0, startY = 0;
    int targetX = 14, targetY = 9;

    std::vector<Node*> openList;
    std::vector<Node*> closedList;
    std::vector<Node*> finalPath;

    openList.push_back(new Node(startX, startY));

    bool pathFound = false;
    bool searchExhausted = false;

    int dx[4] = { 0, 0, -1, 1 };
    int dy[4] = { -1, 1, 0, 0 };

    while (window.isOpen())
    {
        while (const std::optional<sf::Event> event = window.pollEvent())
        {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // --- THE REAL-TIME A* STEP ---
        // Instead of a massive while loop, we process exactly ONE node every frame
        if (!pathFound && !searchExhausted) {
            if (!openList.empty()) {
                Node* currentNode = openList[0];
                int currentIndex = 0;

                for (int i = 1; i < openList.size(); i++) {
                    if (openList[i]->fCost() < currentNode->fCost()) {
                        currentNode = openList[i];
                        currentIndex = i;
                    }
                }

                openList.erase(openList.begin() + currentIndex);
                closedList.push_back(currentNode);

                if (currentNode->x == targetX && currentNode->y == targetY) {
                    Node* trace = currentNode;
                    while (trace != nullptr) {
                        finalPath.push_back(trace);
                        trace = trace->parent;
                    }
                    pathFound = true;
                }

                if (!pathFound) {
                    for (int i = 0; i < 4; i++) {
                        int newX = currentNode->x + dx[i];
                        int newY = currentNode->y + dy[i];

                        if (newX < 0 || newX >= grid[0].size() || newY < 0 || newY >= grid.size()) continue;
                        if (grid[newY][newX] == WALL) continue;

                        bool inClosed = false;
                        for (Node* closedNode : closedList) {
                            if (closedNode->x == newX && closedNode->y == newY) { inClosed = true; break; }
                        }
                        if (inClosed) continue;

                        Node* neighbor = new Node(newX, newY);
                        neighbor->gCost = currentNode->gCost + 1;
                        neighbor->hCost = getDistance(newX, newY, targetX, targetY);
                        neighbor->parent = currentNode;

                        bool inOpen = false;
                        for (Node* openNode : openList) {
                            if (openNode->x == newX && openNode->y == newY) { inOpen = true; break; }
                        }

                        if (!inOpen) {
                            openList.push_back(neighbor);
                        }
                        else {
                            delete neighbor;
                        }
                    }
                }
            }
            else {
                searchExhausted = true;
            }
        }

        // --- DRAW THE STATE ---
        window.clear(sf::Color::Black);

        for (int y = 0; y < grid.size(); y++) {
            for (int x = 0; x < grid[y].size(); x++) {
                sf::RectangleShape cell({ cellSize - 2.0f, cellSize - 2.0f });
                cell.setPosition({ x * cellSize, y * cellSize });

                // Default grid colors
                if (grid[y][x] == EMPTY) cell.setFillColor(sf::Color::White);
                else if (grid[y][x] == WALL) cell.setFillColor(sf::Color(100, 100, 100));

                // Color the Open List (Nodes currently being evaluated) - Yellow
                if (grid[y][x] == EMPTY) {
                    for (Node* n : openList) {
                        if (n->x == x && n->y == y) cell.setFillColor(sf::Color::Yellow);
                    }
                }

                // Color the Closed List (Nodes already evaluated) - Cyan
                if (grid[y][x] == EMPTY) {
                    for (Node* n : closedList) {
                        if (n->x == x && n->y == y) cell.setFillColor(sf::Color::Cyan);
                    }
                }

                // Color the Final Path - Solid Blue
                for (Node* n : finalPath) {
                    if (n->x == x && n->y == y) cell.setFillColor(sf::Color::Blue);
                }

                // Start and Target override everything so they are always visible
                if (grid[y][x] == START) cell.setFillColor(sf::Color::Green);
                else if (grid[y][x] == TARGET) cell.setFillColor(sf::Color::Red);

                window.draw(cell);
            }
        }

        window.display();
    }

    return 0;
}
