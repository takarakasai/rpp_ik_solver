
#include <gtest/gtest.h>

#include <cmath>

#include "rpp_ik_solver/rpp_ik_solver.h"

using namespace Dp;

class RppIKSolverTest : public ::testing::Test{};

const double kDELTA = 0.0001; /* == 0.1 [mm] */

#define D2R(degree) (degree * M_PI / 180.0)

#define CHECK_ACCEPTABLE(var, expect, delta) \
    do {                                     \
      EXPECT_GE(var, expect - delta);        \
      EXPECT_LE(var, expect + delta);        \
    } while(0)

TEST(RppIKSolverTest, IK_unreachable)
{
  RppIKSolver ik(0.022, 0.100, 0.100);
  EXPECT_FALSE(ik.Solve(/*x*/0.022, /*y*/0.0, /*z*/-0.21));
}

TEST(RppIKSolverTest, IK_1)
{
  RppIKSolver ik(0.022, 0.100, 0.100);
  EXPECT_TRUE(ik.Solve(/*x*/0.022, /*y*/0.0, /*z*/-0.2));
  EXPECT_EQ(1, ik.solutions_.size());
  CHECK_ACCEPTABLE(ik.solutions_[0][0], 0, kDELTA);
  CHECK_ACCEPTABLE(ik.solutions_[0][1], 0, kDELTA);
  CHECK_ACCEPTABLE(ik.solutions_[0][2], 0, kDELTA);
}

TEST(RppIKSolverTest, IK_2)
{
  RppIKSolver ik(0.022, 0.100, 0.100);
  EXPECT_TRUE(ik.Solve(/*x*/0.022, /*y*/0.0, /*z*/-0.141421356));
  EXPECT_EQ(2, ik.solutions_.size());
  /* solution #1 */
  CHECK_ACCEPTABLE(ik.solutions_[0][0], 0, kDELTA);
  CHECK_ACCEPTABLE(ik.solutions_[0][1], D2R(+45.0), kDELTA);
  CHECK_ACCEPTABLE(ik.solutions_[0][2], D2R(-90.0), kDELTA);
  /* solution #2 */
  CHECK_ACCEPTABLE(ik.solutions_[1][0], 0, kDELTA);
  CHECK_ACCEPTABLE(ik.solutions_[1][1], D2R(-45.0), kDELTA);
  CHECK_ACCEPTABLE(ik.solutions_[1][2], D2R(+90.0), kDELTA);
}
