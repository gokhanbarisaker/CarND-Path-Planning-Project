#include "behavior.hpp"

const double Behavior::TARGET_SPEED = 50.0;
const double Behavior::MAX_ACCELERATION = 10.0;
const double Behavior::MAX_JERK = 10.0;

std::set<State> Behavior::get_successor_states(const State &state) {
  // TODO: Implement TEST CASES
  // TODO: Implement class

  switch (state)
  {
  case KL: {
    return {
      KL,
      PLCL,
      PLCR,

    };
  }
  // TODO: Consider this from udacity "If state is "LCL" or "LCR", then just return "KL""
  case LCL: {
    return {
      KL,
      LCL,

    };
  }
  case LCR: {
    return {
      KL,
      LCR,

    };
  }
  case PLCL: {
    return {
      KL,
      LCL,
      PLCL,

    };
  }
  case PLCR: {
    return {
      KL,
      LCR,
      PLCR,

    };
  }
  default: {
    // TODO: It might make sense to throw an error at this stage
    return {};
  }
  }
}