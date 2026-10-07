#include "dwa/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace dwa {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Evenly spaced sample i of n across [lo, hi]. Integer indexing avoids the
// drift and resolution-dependent sample counts of a floating-point loop
// counter.
double sampleAt(double lo, double hi, int i, int n) {
  if (n <= 1) {
    return 0.5 * (lo + hi);
  }
  return lo + (hi - lo) * static_cast<double>(i) / static_cast<double>(n - 1);
}

// Observed [min, max] of one cost term across the candidate set, used to map
// that term onto [0, 1].
struct Range {
  double lo = kInf;
  double hi = -kInf;

  void extend(double value) {
    lo = std::min(lo, value);
    hi = std::max(hi, value);
  }

  // Degenerate range (every candidate equal) contributes nothing, rather than
  // amplifying floating-point noise to full scale.
  double normalize(double value) const {
    const double span = hi - lo;
    return span > 1e-12 ? (value - lo) / span : 0.0;
  }
};

}  // namespace

DwaPlanner::DwaPlanner(PlannerConfig config, std::vector<Obstacle> obstacles)
    : config_(std::move(config)), obstacles_(std::move(obstacles)) {}

void DwaPlanner::step(const Control& control, State& state, double dt) {
  state(kTheta) = wrapToPi(state(kTheta) + control(1) * dt);
  state(kX) += control(0) * std::cos(state(kTheta)) * dt;
  state(kY) += control(0) * std::sin(state(kTheta)) * dt;
  state(kV) = control(0);
  state(kOmega) = control(1);
}

DynamicWindowBounds DwaPlanner::dynamicWindow(const State& state) const {
  const double v = state(kV);
  const double omega = state(kOmega);
  const double dv = config_.maxLinearAcceleration * config_.dt;
  const double dw = config_.maxAngularAcceleration * config_.dt;

  DynamicWindowBounds bounds;
  bounds.vMin = std::max(v - dv, config_.minLinearVelocity);
  bounds.vMax = std::min(v + dv, config_.maxLinearVelocity);
  bounds.wMin = std::max(omega - dw, -config_.maxAngularVelocity);
  bounds.wMax = std::min(omega + dw, config_.maxAngularVelocity);
  return bounds;
}

Trajectory DwaPlanner::rollout(const State& start, const Control& control) const {
  const int steps = std::max(
      1, static_cast<int>(std::lround(config_.predictionHorizon / config_.dt)));

  Trajectory trajectory;
  trajectory.reserve(static_cast<std::size_t>(steps) + 1);

  State state = start;
  trajectory.push_back(state);
  for (int i = 0; i < steps; ++i) {
    step(control, state, config_.dt);
    trajectory.push_back(state);
  }
  return trajectory;
}

double DwaPlanner::goalDistanceCost(const Trajectory& trajectory,
                                    const Eigen::Vector2d& goal) const {
  if (trajectory.empty()) {
    return kInf;
  }
  const State& end = trajectory.back();
  const double dx = end(kX) - goal.x();
  const double dy = end(kY) - goal.y();
  return std::sqrt(dx * dx + dy * dy);
}

double DwaPlanner::goalHeadingCost(const Trajectory& trajectory,
                                   const Eigen::Vector2d& goal) const {
  if (trajectory.empty()) {
    return kInf;
  }
  const State& end = trajectory.back();

  // Bearing to the goal measured *from the robot*. The previous version took
  // the angle between the origin->goal and origin->endpoint vectors, which
  // made the cost depend on where the world origin happened to sit.
  const double bearing = std::atan2(goal.y() - end(kY), goal.x() - end(kX));
  return std::abs(angleDiff(bearing, end(kTheta)));
}

bool DwaPlanner::collides(const Trajectory& trajectory) const {
  const double contact = config_.robotRadius + config_.obstacleRadius;
  const double contactSquared = contact * contact;

  for (const State& pose : trajectory) {
    for (const Obstacle& obstacle : obstacles_) {
      const double dx = pose(kX) - obstacle.x;
      const double dy = pose(kY) - obstacle.y;
      if (dx * dx + dy * dy <= contactSquared) {
        return true;
      }
    }
  }
  return false;
}

double DwaPlanner::clearanceCost(const Trajectory& trajectory) const {
  const double contact = config_.robotRadius + config_.obstacleRadius;
  const double contactSquared = contact * contact;

  double minSeparationSquared = kInf;

  // Every pose, every obstacle. An earlier version stepped the trajectory by 2,
  // which let the robot tunnel through thin obstacles.
  for (const State& pose : trajectory) {
    for (const Obstacle& obstacle : obstacles_) {
      const double dx = pose(kX) - obstacle.x;
      const double dy = pose(kY) - obstacle.y;
      const double separationSquared = dx * dx + dy * dy;

      if (separationSquared <= contactSquared) {
        return kInf;  // collision
      }
      minSeparationSquared = std::min(minSeparationSquared, separationSquared);
    }
  }

  if (!std::isfinite(minSeparationSquared)) {
    return 0.0;  // no obstacles at all -- unobstructed
  }
  return 1.0 / std::sqrt(minSeparationSquared);
}

double DwaPlanner::velocityCost(const Trajectory& trajectory) const {
  if (trajectory.empty()) {
    return kInf;
  }
  // Non-negative because the sampled velocity never exceeds the limit.
  return std::max(0.0, config_.maxLinearVelocity - trajectory.back()(kV));
}

std::optional<PlanResult> DwaPlanner::plan(const State& state,
                                           const Eigen::Vector2d& goal) const {
  const DynamicWindowBounds window = dynamicWindow(state);

  struct Candidate {
    Control control;
    Trajectory trajectory;
    double goalDistance;
    double heading;
    double clearance;
    double velocity;
  };

  const int nv = std::max(1, config_.linearVelocitySamples);
  const int nw = std::max(1, config_.angularVelocitySamples);

  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<std::size_t>(nv) * static_cast<std::size_t>(nw));

  for (int iv = 0; iv < nv; ++iv) {
    const double v = sampleAt(window.vMin, window.vMax, iv, nv);
    for (int iw = 0; iw < nw; ++iw) {
      const double omega = sampleAt(window.wMin, window.wMax, iw, nw);

      const Control control(v, omega);
      Trajectory trajectory = rollout(state, control);

      const double clearance = clearanceCost(trajectory);
      if (!std::isfinite(clearance)) {
        continue;  // collides; excluded before normalization
      }

      const double goalDistance = goalDistanceCost(trajectory, goal);
      const double heading = goalHeadingCost(trajectory, goal);
      const double velocity = velocityCost(trajectory);
      if (!std::isfinite(goalDistance) || !std::isfinite(heading) ||
          !std::isfinite(velocity)) {
        continue;
      }

      candidates.push_back(Candidate{control, std::move(trajectory), goalDistance,
                                     heading, clearance, velocity});
    }
  }

  if (candidates.empty()) {
    return std::nullopt;
  }

  // Min-max normalize each term across the candidate set, so every term
  // contributes weight * [0, 1] and the weights actually express relative
  // priority.
  //
  // Dividing by the sum instead -- the other obvious choice -- does not work
  // here: it preserves each term's *relative* spread, so a term with a large
  // mean and small spread (goal distance, ~27 m +/- 0.16 m over a window this
  // narrow) is diluted into irrelevance while one with a small mean and large
  // relative spread (heading error) dominates by two orders of magnitude. The
  // robot then turns to face the goal and never accelerates.
  Range goalDistanceRange;
  Range headingRange;
  Range clearanceRange;
  Range velocityRange;
  for (const Candidate& candidate : candidates) {
    goalDistanceRange.extend(candidate.goalDistance);
    headingRange.extend(candidate.heading);
    clearanceRange.extend(candidate.clearance);
    velocityRange.extend(candidate.velocity);
  }

  double bestCost = kInf;
  const Candidate* best = nullptr;
  for (const Candidate& candidate : candidates) {
    const double cost =
        config_.goalDistanceWeight * goalDistanceRange.normalize(candidate.goalDistance) +
        config_.goalHeadingWeight * headingRange.normalize(candidate.heading) +
        config_.obstacleWeight * clearanceRange.normalize(candidate.clearance) +
        config_.velocityWeight * velocityRange.normalize(candidate.velocity);

    if (cost < bestCost) {
      bestCost = cost;
      best = &candidate;
    }
  }

  if (best == nullptr) {
    return std::nullopt;  // every cost was NaN
  }

  PlanResult result;
  result.control = best->control;
  result.trajectory = best->trajectory;
  return result;
}

Control DwaPlanner::recoveryControl(const State& state) const {
  const DynamicWindowBounds window = dynamicWindow(state);

  // Decelerate toward zero and rotate in place, both clamped to what the
  // acceleration limits actually permit this step.
  const double v = std::clamp(0.0, window.vMin, window.vMax);
  const double omega =
      std::clamp(config_.maxAngularVelocity, window.wMin, window.wMax);
  return Control(v, omega);
}

bool DwaPlanner::atGoal(const State& state, const Eigen::Vector2d& goal) const {
  const double dx = state(kX) - goal.x();
  const double dy = state(kY) - goal.y();
  // Squared distance against squared tolerance. An earlier version compared a
  // squared distance against an unsquared radius, which was only correct
  // because that radius happened to be 1.
  return (dx * dx + dy * dy) <= config_.goalTolerance * config_.goalTolerance;
}

}  // namespace dwa
