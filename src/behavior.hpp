#ifndef PP_BEHAVIOR_H
#define PP_BEHAVIOR_H

#include <set>
#include "state.hpp"
#include "car.hpp"
#include <vector>

class Behavior
{
public:
  Behavior() {}
  virtual ~Behavior() {}

  const std::set<State> get_successor_states(const State &state);
  const State get_successor_state(const State &state, const MainCar &main_car, const std::vector<OtherCar> &other_cars);

private:
  double get_cost(const State &state, const int lane, const double main_car_s, const double main_car_speed, const double other_car_s, const double other_car_speed, const Position &nearby_car_positon);
  double get_speed_cost(const State &state, const double main_car_speed, const double other_car_speed, const Position &other_car_position);
  double get_lane_cost(const State &state, const int lane, const Position &other_car_position);
};

#endif