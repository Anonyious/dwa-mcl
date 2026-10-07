#ifndef DWA_PLANNER_HPP_
#define DWA_PLANNER_HPP_

#include <optional>
#include <vector>

#include "dwa/types.hpp"

namespace dwa {

// Dynamic Window Approach local planner.
//
// Fox, Burgard & Thrun, "The Dynamic Window Approach to Collision Avoidance",
// IEEE Robotics & Automation Magazine 4(1), 1997.
//
// Deliberately free of any rendering or I/O dependency so every cost term and
// the window derivation can be unit-tested directly.
class DwaPlanner {
 public:
  DwaPlanner(PlannerConfig config, std::vector<Obstacle> obstacles);

  // Admissible velocity window reachable within one dt under the
  // acceleration limits, clamped to the absolute velocity limits.
  DynamicWindowBounds dynamicWindow(const State& state) const;

  // Forward-simulate a constant control over the prediction horizon.
  Trajectory rollout(const State& state, const Control& control) const;

  // Pick the lowest-cost collision-free trajectory.
  // Returns nullopt when every sampled trajectory collides -- callers must
  // handle that case (see recoveryControl) rather than indexing the result.
  std::optional<PlanResult> plan(const State& state,
                                 const Eigen::Vector2d& goal) const;

  // Control to issue when plan() finds nothing: decelerate toward zero and
  // rotate in place, both clamped to the admissible window.
  Control recoveryControl(const State& state) const;

  // --- Cost terms, public for testing. All are non-negative. ---

  // Euclidean distance from the trajectory endpoint to the goal.
  double goalDistanceCost(const Trajectory& trajectory,
                          const Eigen::Vector2d& goal) const;

  // Absolute bearing error at the endpoint: the angle between the robot's
  // heading and the direction to the goal, both measured *at the robot*.
  double goalHeadingCost(const Trajectory& trajectory,
                         const Eigen::Vector2d& goal) const;

  // 1 / (closest approach to any obstacle), or +infinity if the trajectory
  // collides. Every pose is checked -- no subsampling.
  double clearanceCost(const Trajectory& trajectory) const;

  // Penalizes falling short of the maximum linear velocity.
  double velocityCost(const Trajectory& trajectory) const;

  // True when the trajectory passes within (robotRadius + obstacleRadius) of
  // any obstacle at any pose.
  bool collides(const Trajectory& trajectory) const;

  // Single Euler step of the unicycle model. Theta is wrapped.
  static void step(const Control& control, State& state, double dt);

  bool atGoal(const State& state, const Eigen::Vector2d& goal) const;

  const PlannerConfig& config() const { return config_; }
  const std::vector<Obstacle>& obstacles() const { return obstacles_; }

 private:
  PlannerConfig config_;
  std::vector<Obstacle> obstacles_;
};

}  // namespace dwa

#endif  // DWA_PLANNER_HPP_
