#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>

struct Point
{
    double x, y;
};

struct Rectangle
{
    double x, y, width, height;
};

bool pointInRectangle(const Point &p, const Rectangle &rect)
{
    return p.x >= rect.x && p.x <= rect.x + rect.width &&
           p.y >= rect.y && p.y <= rect.y + rect.height;
}

double distance(const Point &p1, const Point &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

int randomInt(int min, int max)
{
    return min + std::rand() % (max - min + 1);
}

struct Node
{
    Point point;
    int parentIndex;
};

class RRT
{
public:
    RRT(double stepSize, int maxIterations)
        : stepSize(stepSize), maxIterations(maxIterations) {}

    void generateRRT(Point start, Point goal, const std::vector<Rectangle> &obstacles)
    {
        tree.push_back({start, -1});

        for (int i = 0; i < maxIterations; ++i)
        {
            Point randomPoint = {static_cast<double>(randomInt(0, 800)), static_cast<double>(randomInt(0, 600))}; // Window size

            int nearestNodeIndex = getNearestNodeIndex(randomPoint);
            Point newPoint = extend(tree[nearestNodeIndex].point, randomPoint);

            if (!collisionWithObstacles(newPoint, obstacles))
            {
                tree.push_back({newPoint, nearestNodeIndex});

                if (distance(newPoint, goal) < stepSize)
                {
                    std::cout << "Path found!" << std::endl;
                    constructPath(goal);
                    break;
                }
            }
        }
    }

    std::vector<Node> getTree() const
    {
        return tree;
    }

    std::vector<Point> getPath() const
    {
        return path;
    }

private:
    double stepSize;
    int maxIterations;
    std::vector<Node> tree;
    std::vector<Point> path;

    int getNearestNodeIndex(const Point &point)
    {
        double minDistance = std::numeric_limits<double>::max();
        int nearestNodeIndex = -1;

        for (int i = 0; i < tree.size(); ++i)
        {
            double d = distance(tree[i].point, point);
            if (d < minDistance)
            {
                minDistance = d;
                nearestNodeIndex = i;
            }
        }

        return nearestNodeIndex;
    }

    Point extend(const Point &from, const Point &to)
    {
        double d = distance(from, to);
        if (d <= stepSize)
        {
            return to;
        }
        else
        {
            double theta = std::atan2(to.y - from.y, to.x - from.x);
            return {from.x + stepSize * std::cos(theta), from.y + stepSize * std::sin(theta)};
        }
    }

    bool collisionWithObstacles(const Point &point, const std::vector<Rectangle> &obstacles)
    {
        for (const auto &obstacle : obstacles)
        {
            if (pointInRectangle(point, obstacle))
            {
                return true;
            }
        }
        return false;
    }

    void constructPath(const Point &goal)
    {
        int currentNodeIndex = tree.size() - 1;
        while (currentNodeIndex != -1)
        {
            path.push_back(tree[currentNodeIndex].point);
            if (distance(tree[currentNodeIndex].point, goal) <= stepSize)
            {
                break;
            }
            currentNodeIndex = tree[currentNodeIndex].parentIndex;
        }
        std::reverse(path.begin(), path.end());
    }
};

int main()
{
    double stepSize = 20.0;
    int maxIterations = 5000;

    Point start = {static_cast<double>(randomInt(0, 800)), static_cast<double>(randomInt(0, 600))};
    Point goal = {static_cast<double>(randomInt(0, 800)), static_cast<double>(randomInt(0, 600))};

    std::cout << "Enter the number of obstacles: ";
    int numObstacles;
    std::cin >> numObstacles;

    std::vector<Rectangle> obstacles;
    for (int i = 0; i < numObstacles; ++i)
    {
        Rectangle obstacle = {
            static_cast<double>(randomInt(0, 800)),
            static_cast<double>(randomInt(0, 600)),
            static_cast<double>(randomInt(10, 50)),
            static_cast<double>(randomInt(10, 50))};
        obstacles.push_back(obstacle);
    }

    RRT rrt(stepSize, maxIterations);
    rrt.generateRRT(start, goal, obstacles);

    sf::RenderWindow window(sf::VideoMode(800, 600), "RRT Visualization");
    window.setFramerateLimit(60);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
        }

        window.clear(sf::Color::Black);

        for (const auto &obstacle : obstacles)
        {
            sf::RectangleShape obstacleRect(sf::Vector2f(obstacle.width, obstacle.height));
            obstacleRect.setPosition(obstacle.x, obstacle.y);
            obstacleRect.setFillColor(sf::Color::Red);
            window.draw(obstacleRect);
        }

        sf::CircleShape startCircle(5);
        startCircle.setPosition(start.x, start.y);
        startCircle.setFillColor(sf::Color::Green);
        window.draw(startCircle);

        sf::CircleShape goalCircle(5);
        goalCircle.setPosition(goal.x, goal.y);
        goalCircle.setFillColor(sf::Color::Blue);
        window.draw(goalCircle);

        for (const auto &node : rrt.getTree())
        {
            if (node.parentIndex != -1)
            {
                sf::Vertex line[] = {
                    sf::Vertex(sf::Vector2f(node.point.x, node.point.y)),
                    sf::Vertex(sf::Vector2f(rrt.getTree()[node.parentIndex].point.x, rrt.getTree()[node.parentIndex].point.y))};
                window.draw(line, 2, sf::Lines);
            }
        }

        for (int i = 0; i < rrt.getPath().size() - 1; ++i)
        {
            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(rrt.getPath()[i].x, rrt.getPath()[i].y), sf::Color::Red),
                sf::Vertex(sf::Vector2f(rrt.getPath()[i + 1].x, rrt.getPath()[i + 1].y), sf::Color::Red)};
            window.draw(line, 2, sf::Lines);
        }
        std::cout << "Path points:" << std::endl;

        window.display();
    }
}