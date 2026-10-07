#ifndef DWA_TYPES_HPP_
#define DWA_TYPES_HPP_

#include <vector>

#include <Eigen/Dense>

#include "dwa/angles.hpp"

namespace dwa {

// Robot state: [x, y, theta, v, omega]. Fixed-size, so it lives on the stack
// and costs nothing to copy -- unlike the VectorXd this used to be.
using State = Eigen::Matrix<double, 5, 1>;

// Control input: [linear velocity, angular velocity].
using Control = Eigen::Vector2d;

using Trajectory = std::vector<State>;

// Indices into State, so call sites stop using bare magic numbers.
enum StateIndex { kX = 0, kY = 1, kTheta = 2, kV = 3, kOmega = 4 };

struct Obstacle {
  double x = 0.0;
  double y = 0.0;
};

// Admissible velocity window for one control step, derived from the current
// velocity and the acceleration limits.
struct DynamicWindowBounds {
  double vMin = 0.0;
  double vMax = 0.0;
  double wMin = 0.0;
  double wMax = 0.0;
};

struct PlannerConfig {
  // --- Kinematic limits ---
  double maxLinearVelocity = 1.0;                      // m/s
  double minLinearVelocity = -0.5;                     // m/s (reverse allowed)
  double maxAngularVelocity = degToRad(40.0);          // rad/s
  double maxLinearAcceleration = 0.2;                  // m/s^2
  double maxAngularAcceleration = degToRad(40.0);      // rad/s^2

  // --- Sampling ---
  double dt = 0.1;                   // s, integration step and control period
  double predictionHorizon = 4.0;    // s
  int linearVelocitySamples = 11;    // integer counts, not float strides
  int angularVelocitySamples = 21;

  // --- Cost weights ---
  // Each term is min-max normalized across the candidate set before
  // weighting, so these are dimensionless and directly comparable: a term
  // contributes weight * [0, 1].
  //
  // Balanced weights are not an arbitrary default. Over-weighting heading
  // (>= ~2x the velocity weight) or clearance makes the robot turn to face
  // the goal and then refuse to accelerate, because the narrow dynamic window
  // means a single cycle barely changes position while it can change heading
  // a lot. See docs/cost_function.md.
  double goalDistanceWeight = 1.0;
  double goalHeadingWeight = 1.0;
  double obstacleWeight = 1.0;
  double velocityWeight = 1.0;

  // --- Geometry ---
  double robotRadius = 1.0;      // m, collision footprint
  double obstacleRadius = 0.0;   // m, obstacle inflation
  double goalTolerance = 1.0;    // m; distinct from robotRadius on purpose

  // --- Safety ---
  int maxIterations = 2000;      // bound the run if the goal is unreachable
};

struct PlanResult {
  Control control = Control::Zero();
  Trajectory trajectory;
};

}  // namespace dwa

#endif  // DWA_TYPES_HPP_
