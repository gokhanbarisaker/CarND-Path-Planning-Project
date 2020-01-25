#include <gtest/gtest.h>
#include <vector>

#include "../src/behavior.hpp"

TEST(Behavior, SuccessorState) {
  Behavior behavior;
  std::set<State> kl_successors = behavior.get_successor_states(KL);
  std::set<State> kl_expected = {
    KL,
    PLCL,
    PLCR
  };

  ASSERT_EQ(kl_successors, kl_expected);

  std::set<State> lcl_successors = behavior.get_successor_states(LCL);
  std::set<State> lcl_expected = {
    LCL,
    KL
  };

  ASSERT_EQ(lcl_successors, lcl_expected);

  std::set<State> lcr_successors = behavior.get_successor_states(LCR);
  std::set<State> lcr_expected = {
    LCR,
    KL
  };

  ASSERT_EQ(lcr_successors, lcr_expected);

  std::set<State> plcl_successors = behavior.get_successor_states(PLCL);
  std::set<State> plcl_expected = {
    PLCL,
    LCL,
    KL
  };
  
  ASSERT_EQ(plcl_successors, plcl_expected);

  std::set<State> plcr_successors = behavior.get_successor_states(PLCR);
  std::set<State> plcr_expected = {
    PLCR,
    LCR,
    KL
  };
  
  ASSERT_EQ(plcr_successors, plcr_expected);
}

TEST(Trajectory, Generate) {
  FAIL();
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
