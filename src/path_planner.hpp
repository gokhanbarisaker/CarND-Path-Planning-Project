#ifndef PP_PATH_PLANNER_H
#define PP_PATH_PLANNER_H

#include "car.hpp"
#include "state.hpp"
#include <utility>
#include <vector>
#include "behavior.hpp"

// TODO: Return best (lowest cost) trajectory

class PathPlanner
{
public:
  PathPlanner(
      std::vector<double> map_waypoints_x,
      std::vector<double> map_waypoints_y,
      std::vector<double> map_waypoints_s,
      std::vector<double> map_waypoints_dx,
      std::vector<double> map_waypoints_dy);
  virtual ~PathPlanner() {}

  // map values for waypoint's x,y,s and d normalized normal vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  const std::pair<std::vector<double>, std::vector<double>> plan(
      const MainCar &main_car,
      const std::vector<OtherCar> &other_cars,
      const std::vector<double> &previous_path_x,
      const std::vector<double> &previous_path_y,
      const double end_path_s,
      const double end_path_d);

private:
  State state = State::KL;
  Behavior behavior;
};

#endif