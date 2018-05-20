#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define NUM_TIMESTEPS                                  (15)   // N = 6
#define DELTA_TIME                                     (0.1) // 100ms for dt
#define REFERENCE_SPEED                                (50 * 0.447)  // ref_v = 50 mph. Convert to m/s.
#define DELTA_ACTUATOR_COST_MULTIPLIER                 (300)  // Tuning parameter for weighing
                                                             // cost of applying steering control
#define DELTA_ACTUATOR_CHANGE_COST_MULTIPLIER          (300)  // Tuning parameter for weighing
                                                             // cost of changing steering control
using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
