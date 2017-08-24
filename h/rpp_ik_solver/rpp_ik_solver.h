
#include <array>
#include <vector>

namespace Dp {

  typedef std::array<double, 3> Vec3d;
  typedef std::vector<Vec3d> RppIKSolutions;
  
  class RppIKSolver {
  public:
    RppIKSolver(double l1, double l2, double l3);
    virtual ~RppIKSolver();
  
    bool Solve(double x, double y, double z);
  
  public:
    RppIKSolutions solutions_;
  
  private:
    static const double kDeltaRadian;

    double l1_; /* roll    --> pitch#1 */
    double l2_; /* pitch#1 --> pitch#2 */
    double l3_; /* pitch#2 --> EE      */
  };

}
