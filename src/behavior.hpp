#ifndef PP_BEHAVIOR_H
#define PP_BEHAVIOR_H

#include <set>

#include "state.hpp"

class Behavior {
  public:
    std::set<State> get_successor_states(const State &state);

    static const double TARGET_SPEED;
    static const double MAX_ACCELERATION;
    static const double MAX_JERK;

};

#endif