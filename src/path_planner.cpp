#include "path_planner.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include <iostream>
#include "trajectory/trajectory.hpp"
#include "trajectory/constants.hpp"

PathPlanner::PathPlanner(
    std::vector<double> map_waypoints_x,
    std::vector<double> map_waypoints_y,
    std::vector<double> map_waypoints_s,
    std::vector<double> map_waypoints_dx,
    std::vector<double> map_waypoints_dy)
{
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
    this->map_waypoints_dx = map_waypoints_dx;
    this->map_waypoints_dy = map_waypoints_dy;
}

const std::pair<std::vector<double>, std::vector<double>> PathPlanner::plan(
    const MainCar &main_car,
    const std::vector<OtherCar> &other_cars,
    const std::vector<double> &previous_path_x,
    const std::vector<double> &previous_path_y,
    const double end_path_s,
    const double end_path_d)
{
    state = behavior.get_successor_state(state, main_car, other_cars);

    return Trajectory::of(
        state,
        main_car,
        other_cars,
        previous_path_x,
        previous_path_y,
        map_waypoints_x,
        map_waypoints_y,
        map_waypoints_s,
        map_waypoints_dx,
        map_waypoints_dy);
}