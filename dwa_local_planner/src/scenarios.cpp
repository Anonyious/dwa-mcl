#include "dwa/scenarios.hpp"

#include <cmath>

namespace dwa {
namespace {

State makeStart(double x, double y, double yawDegrees) {
  State start = State::Zero();
  start(kX) = x;
  start(kY) = y;
  start(kTheta) = degToRad(yawDegrees);
  return start;
}

// The obstacle field this planner was first built against.
Scenario defaultScenario() {
  Scenario scenario;
  scenario.name = "default";
  scenario.description = "Original demo field; start and goal straddle the origin.";
  scenario.start = makeStart(-13.0, -8.0, 22.5);
  scenario.goal = Eigen::Vector2d(10.0, 15.0);
  scenario.obstacles = {
      {-8.0, -5.0},  {-8.0, 11.0}, {-13.0, 10.0}, {-13.0, 5.0}, {0.0, 5.0},
      {-5.0, 5.0},   {5.0, 11.0},  {-11.0, 0.0},  {-1.0, -1.0}, {0.0, 2.0},
      {4.0, 2.0},    {5.0, 15.0},  {8.0, 9.0},    {12.0, 12.0},
  };
  return scenario;
}

// Same obstacle field, but start and goal both sit in the first quadrant so
// neither the origin nor the origin-goal ray lies between them. The old
// origin-referenced heading cost misbehaved here; the robot-relative one does
// not. Kept as a regression scenario.
Scenario offsetScenario() {
  Scenario scenario;
  scenario.name = "offset";
  scenario.description =
      "Start and goal both far from the origin; regression case for the "
      "robot-relative heading cost.";
  scenario.start = makeStart(22.0, 22.0, 180.0);
  scenario.goal = Eigen::Vector2d(40.0, 34.0);
  scenario.obstacles = {
      {28.0, 24.0}, {30.0, 30.0}, {33.0, 27.0}, {26.0, 31.0},
      {35.0, 32.0}, {31.0, 22.0}, {37.0, 29.0},
  };
  return scenario;
}

// The robot starts already inside the collision radius of an obstacle, so
// every sampled rollout -- including pure in-place rotation -- collides and
// plan() has nothing to return. Exercises the recovery path; an earlier version
// code indexed into the resulting empty trajectory.
Scenario trappedScenario() {
  Scenario scenario;
  scenario.name = "trapped";
  scenario.description =
      "Robot starts inside an obstacle's collision radius; every rollout "
      "collides, so the planner must fall back to recovery.";
  scenario.start = makeStart(0.0, 0.0, 0.0);
  scenario.goal = Eigen::Vector2d(10.0, 10.0);
  scenario.obstacles = {{0.6, 0.0}};
  return scenario;
}

// A corridor sealed by a cross wall. Obstacle spacing (0.5 m) is well below
// the collision radius, so there is no gap to slip through, and the side
// walls mean there is no way around either -- a plain finite wall can simply
// be driven around, which is not a tunneling failure.
//
// The goal sits beyond the wall and is unreachable by construction. The robot
// must approach, stop, and never appear on the far side. An earlier version cost
// function sampled every other trajectory pose and could step across a thin
// obstacle without ever registering a collision.
Scenario corridorScenario() {
  Scenario scenario;
  scenario.name = "corridor";
  scenario.description =
      "Dead-end corridor sealed by a wall; goal is unreachable and the robot "
      "must never cross to the far side.";
  scenario.start = makeStart(0.0, -6.0, 90.0);
  scenario.goal = Eigen::Vector2d(0.0, 8.0);

  // Side walls, y in [-9, 0].
  for (int i = -18; i <= 1; ++i) {
    const double y = 0.5 * static_cast<double>(i);
    scenario.obstacles.push_back(Obstacle{-3.0, y});
    scenario.obstacles.push_back(Obstacle{3.0, y});
  }
  // Cross wall sealing the corridor at y = 0.
  for (int i = -6; i <= 6; ++i) {
    scenario.obstacles.push_back(Obstacle{0.5 * static_cast<double>(i), 0.0});
  }
  return scenario;
}

}  // namespace

std::optional<Scenario> makeScenario(const std::string& name) {
  if (name == "default") {
    return defaultScenario();
  }
  if (name == "offset") {
    return offsetScenario();
  }
  if (name == "trapped") {
    return trappedScenario();
  }
  if (name == "corridor") {
    return corridorScenario();
  }
  return std::nullopt;
}

std::vector<std::string> scenarioNames() {
  return {"default", "offset", "trapped", "corridor"};
}

}  // namespace dwa
