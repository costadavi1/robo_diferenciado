#pragma once

#ifndef ROBO_DIFERENCIADO_A_STAR_HPP
#define ROBO_DIFERENCIADO_A_STAR_HPP

#include <iostream>
#include <queue>
#include <vector>
#include <chrono>
// #include <thread>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <nav_msgs/msg/occupancy_grid.hpp>

class AStar
{
public:
    bool import_map(const nav_msgs::msg::OccupancyGrid::SharedPtr &map);
    bool plan_path(const pair<double, double> &start, const pair<double, double> &goal, vector<pair<double, double>> &result);

private:
    double euclidian_distance(const std::pair<int, int> &goal, const std::pair<int, int> &current);

    int rows_;
    int cols_;
    vector<vector<int>> grid_;
};
