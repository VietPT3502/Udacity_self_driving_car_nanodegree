#include "cost.h"
double inefficiency_cost(int target_speed, int intended_lane, int final_lane, 
                         const std::vector<int> &lane_speeds) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than target_speed.

  /**
   * TODO: Replace cost = 0 with an appropriate cost function.
   */
  double cost = (2.0 * target_speed - lane_speeds[intended_lane] - lane_speeds[final_lane]) / target_speed;
  return cost;
}