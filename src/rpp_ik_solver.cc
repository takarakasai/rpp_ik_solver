
#include "rpp_ik_solver/rpp_ik_solver.h"

#include <cmath>

#if defined(DP_RPP_IK_SOLVER_DEBUG)
  #include <iostream>
#endif

namespace Dp {

  #define R2D(radians) (radians * 180.0 / M_PI)
  #define D2R(degrees) (degrees * M_PI / 180.0)

  const double RppIKSolver::kDeltaRadian = D2R(0.0001);

  RppIKSolver::RppIKSolver(double l1, double l2, double l3, double offset[3]) {

    solutions_.reserve(2);
    solutions_.clear();
  
    l1_ = l1;
    l2_ = l2;
    l3_ = l3;

    for (size_t i = 0; i < 3; I++) {
      offset_[i] = offset[i];
    }

    return;
  }

  RppIKSolver::~RppIKSolver() {
    return;
  }
  
  bool RppIKSolver::Solve(double x, double y, double z) {

    x -= offset[0];
    y -= offset[1];
    z -= offset[2];

    solutions_.clear();
  
    double roll = atan2(y, -z);
  
    double r_reachable = l2_ + l3_;
    double r_xyz_2 = pow(x - l1_, 2) + pow(y, 2) + pow(z, 2);
    double r_xyz   = sqrt(r_xyz_2);
  
    if (r_xyz > r_reachable) {
      return false;
    }

    double r_yz_2  = pow(y, 2) + pow(z, 2);
    double r_yz    = sqrt(r_yz_2);
  
    double tmp1 = (pow(l2_, 2) + pow(l3_, 2) - r_xyz_2) / (2 * l2_ * l3_);
    double ang_tri_2 = acos(tmp1);
  
    double tmp2 = (pow(l2_, 2) + r_xyz_2 - pow(l3_, 2)) / (2 * l2_ * r_xyz);
    double ang_tri_1 = acos(tmp2);
    if (ang_tri_1 >= M_PI) {
      ang_tri_1 = M_PI - ang_tri_1;
    }
  
    double pitch2[2] = {
        -(M_PI - ang_tri_2),
         (M_PI - ang_tri_2)
    };
  
    //double z_dash = z > 0 ? -r_yz : r_yz;
    double z_dash = r_yz;
    double ang_pitch_all = atan2(x - l1_, z_dash);
  
    double pitch1[2] = {
        -(ang_pitch_all - ang_tri_1),
        -(ang_pitch_all + ang_tri_1),
    };
 
    solutions_.push_back(Vec3d({roll, pitch1[0], pitch2[0]}));
    if ((pitch1[0] < (pitch1[1] - kDeltaRadian)) || ((pitch1[1] + kDeltaRadian) < pitch1[0])) {
      solutions_.push_back(Vec3d({roll, pitch1[1], pitch2[1]}));
    }
 
#if defined(DP_RPP_IK_SOLVER_DEBUG)
    std::cout << "roll    : " << roll << " [rad], " << R2D(roll) << " [deg]"  << std::endl;
    std::cout << "r_xyz_2 : " << r_xyz_2 << std::endl;
    std::cout << "r_xyz   : " << r_xyz   << std::endl;
    std::cout << "r_yz    : " << r_yz    << std::endl;
    std::cout << "tmp1    : " << tmp1       << std::endl;
    std::cout << "tri2    : " << ang_tri_2 << " [rad], " << R2D(ang_tri_2) << " [deg]" << std::endl;
    std::cout << "tmp2    : " << tmp2      << std::endl;
    std::cout << "tri1    : " << ang_tri_1 << " [rad], " << R2D(ang_tri_1) << " [deg]"  << std::endl;
    std::cout << "pitch2 1: " << pitch2[0] << " [rad], " << R2D(pitch2[0]) << " [deg]"  << std::endl;
    std::cout << "pitch2 2: " << pitch2[1] << " [rad], " << R2D(pitch2[1]) << " [deg]"  << std::endl;
    std::cout << "x - l1_ : " << x - l1_ << std::endl;
    std::cout << "z_dash  : " << z_dash  << std::endl;
    std::cout << "all     : " << ang_pitch_all << " [rad], " << R2D(ang_pitch_all) << " [deg]"  << std::endl;
    std::cout << "pitch1 1: " << pitch1[0] << " [rad], " << R2D(pitch1[0]) << " [deg]"  << std::endl;
    std::cout << "pitch1 2: " << pitch1[1] << " [rad], " << R2D(pitch1[1]) << " [deg]"  << std::endl;
    //std::cout << " --> " << R2D(solutions_[0]) << ", " << R2D(solutions_[1]) << ", " << R2D(solutions_[2]) << std::endl;
#endif
  
    return true;
  }
}
