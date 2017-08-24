
#include "rpp_ik_solver/rpp_ik_solver.h"

using namespace Dp;

#include <iostream>
#include <cstdlib>

int main(int argc, char *argv[]) {
  if (argc < 4) {
      return 0;
  }

  double pos[] = {
    strtod(argv[1], NULL),
    strtod(argv[2], NULL),
    strtod(argv[3], NULL),
  };

  std::cout << " target : " <<  pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;

  RppIKSolver ik(0.0, 0.10, 0.10);
  ik.Solve(pos[0], pos[1], pos[2]);

  return 0;
}

