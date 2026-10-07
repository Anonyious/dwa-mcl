#include "dwa/planner.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "dwa/scenarios.hpp"

namespace {

dwa::State poseAt(double x, double y, double theta, double v = 0.0,
                  double omega = 0.0) {
  dwa::State state = dwa::State::Zero();
  state(dwa::kX) = x;
  state(dwa::kY) = y;
  state(dwa::kTheta) = theta;
  state(dwa::kV) = v;
  state(dwa::kOmega) = omega;
  return state;
}

dwa::DwaPlanner makePlanner(std::vector<dwa::Obstacle> obstacles = {},
                            dwa::PlannerConfig config = {}) {
  return dwa::DwaPlanner(config, std::move(obstacles));
}

// --- Dynamic window ---

TEST(DynamicWindow, IsCenteredOnCurrentVelocity) {
  dwa::PlannerConfig config;
  config.dt = 0.1;
  config.maxLinearAcceleration = 0.2;
  config.maxAngularAcceleration = 0.4;
  const dwa::DwaPlanner planner = makePlanner({}, config);

  const dwa::DynamicWindowBounds window = planner.dynamicWindow(poseAt(0, 0, 0, 0.5, 0.1));

  EXPECT_NEAR(window.vMin, 0.5 - 0.02, 1e-12);
  EXPECT_NEAR(window.vMax, 0.5 + 0.02, 1e-12);
  EXPECT_NEAR(window.wMin, 0.1 - 0.04, 1e-12);
  EXPECT_NEAR(window.wMax, 0.1 + 0.04, 1e-12);
}

TEST(DynamicWindow, ClampsToAbsoluteLimits) {
  dwa::PlannerConfig config;
  config.maxLinearVelocity = 1.0;
  config.minLinearVelocity = -0.5;
  config.maxAngularVelocity = 0.3;
  const dwa::DwaPlanner planner = makePlanner({}, config);

  // At the top speed the window cannot exceed the absolute limit.
  const dwa::DynamicWindowBounds high = planner.dynamicWindow(poseAt(0, 0, 0, 1.0, 0.3));
  EXPECT_DOUBLE_EQ(high.vMax, 1.0);
  EXPECT_DOUBLE_EQ(high.wMax, 0.3);

  const dwa::DynamicWindowBounds low = planner.dynamicWindow(poseAt(0, 0, 0, -0.5, -0.3));
  EXPECT_DOUBLE_EQ(low.vMin, -0.5);
  EXPECT_DOUBLE_EQ(low.wMin, -0.3);
}

// --- Motion model ---

TEST(Step, DrivesStraightAlongHeading) {
  dwa::State state = poseAt(0, 0, 0);
  dwa::DwaPlanner::step(dwa::Control(1.0, 0.0), state, 0.5);

  EXPECT_NEAR(state(dwa::kX), 0.5, 1e-12);
  EXPECT_NEAR(state(dwa::kY), 0.0, 1e-12);
  EXPECT_NEAR(state(dwa::kTheta), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(state(dwa::kV), 1.0);
}

TEST(Step, RotationInPlaceDoesNotTranslate) {
  dwa::State state = poseAt(3.0, 4.0, 0.0);
  for (int i = 0; i < 100; ++i) {
    dwa::DwaPlanner::step(dwa::Control(0.0, 0.5), state, 0.1);
  }
  EXPECT_NEAR(state(dwa::kX), 3.0, 1e-12);
  EXPECT_NEAR(state(dwa::kY), 4.0, 1e-12);
}

TEST(Step, KeepsThetaWrapped) {
  dwa::State state = poseAt(0, 0, 0);
  for (int i = 0; i < 500; ++i) {
    dwa::DwaPlanner::step(dwa::Control(0.0, 1.0), state, 0.1);
    EXPECT_LE(state(dwa::kTheta), dwa::kPi + 1e-9);
    EXPECT_GE(state(dwa::kTheta), -dwa::kPi - 1e-9);
  }
}

TEST(Rollout, HasHorizonOverDtPlusOnePoses) {
  dwa::PlannerConfig config;
  config.dt = 0.1;
  config.predictionHorizon = 4.0;
  const dwa::DwaPlanner planner = makePlanner({}, config);

  const dwa::Trajectory trajectory = planner.rollout(poseAt(0, 0, 0), dwa::Control(1.0, 0.0));
  EXPECT_EQ(trajectory.size(), 41u);
  EXPECT_NEAR(trajectory.back()(dwa::kX), 4.0, 1e-9);
}

// --- Goal test ---

// Regression: an earlier version compared a squared distance against an unsquared
// radius inside sqrt(), which was correct only because the radius was 1.
TEST(AtGoal, UsesToleranceNotItsSquareRoot) {
  dwa::PlannerConfig config;
  config.goalTolerance = 2.0;
  const dwa::DwaPlanner planner = makePlanner({}, config);
  const Eigen::Vector2d goal(0.0, 0.0);

  EXPECT_TRUE(planner.atGoal(poseAt(1.9, 0.0, 0.0), goal));
  EXPECT_TRUE(planner.atGoal(poseAt(2.0, 0.0, 0.0), goal));
  EXPECT_FALSE(planner.atGoal(poseAt(2.1, 0.0, 0.0), goal));

  // sqrt(2) ~= 1.414 is what the buggy form would have accepted as the edge.
  EXPECT_TRUE(planner.atGoal(poseAt(1.5, 0.0, 0.0), goal));
}

TEST(AtGoal, ToleranceIsIndependentOfRobotRadius) {
  dwa::PlannerConfig config;
  config.goalTolerance = 0.5;
  config.robotRadius = 5.0;
  const dwa::DwaPlanner planner = makePlanner({}, config);

  EXPECT_FALSE(planner.atGoal(poseAt(1.0, 0.0, 0.0), Eigen::Vector2d(0.0, 0.0)));
}

// --- Heading cost ---

// Regression: the old cost measured the angle between the origin->goal and
// origin->endpoint vectors, so it depended on where the world origin sat.
// A robot-relative bearing error must not.
TEST(GoalHeadingCost, IsInvariantUnderWorldTranslation) {
  const dwa::DwaPlanner planner = makePlanner();

  const dwa::Trajectory nearOrigin = planner.rollout(poseAt(0.0, 0.0, 0.3), dwa::Control(1.0, 0.0));
  const double costNearOrigin = planner.goalHeadingCost(nearOrigin, Eigen::Vector2d(10.0, 2.0));

  const double shiftX = 1000.0;
  const double shiftY = -500.0;
  const dwa::Trajectory shifted =
      planner.rollout(poseAt(shiftX, shiftY, 0.3), dwa::Control(1.0, 0.0));
  const double costShifted =
      planner.goalHeadingCost(shifted, Eigen::Vector2d(10.0 + shiftX, 2.0 + shiftY));

  EXPECT_NEAR(costNearOrigin, costShifted, 1e-9);
}

TEST(GoalHeadingCost, IsZeroWhenAimedAtTheGoal) {
  const dwa::DwaPlanner planner = makePlanner();
  const dwa::Trajectory trajectory = planner.rollout(poseAt(0, 0, 0), dwa::Control(0.0, 0.0));
  EXPECT_NEAR(planner.goalHeadingCost(trajectory, Eigen::Vector2d(5.0, 0.0)), 0.0, 1e-12);
}

TEST(GoalHeadingCost, IsPiWhenFacingDirectlyAway) {
  const dwa::DwaPlanner planner = makePlanner();
  const dwa::Trajectory trajectory = planner.rollout(poseAt(0, 0, 0), dwa::Control(0.0, 0.0));
  EXPECT_NEAR(planner.goalHeadingCost(trajectory, Eigen::Vector2d(-5.0, 0.0)), dwa::kPi, 1e-12);
}

TEST(GoalHeadingCost, IsFiniteAndNonNegativeEverywhere) {
  const dwa::DwaPlanner planner = makePlanner();
  // Includes the endpoint sitting exactly on the goal and exactly on the
  // origin, both of which produced NaN in the acos-based version.
  const std::vector<Eigen::Vector2d> goals = {
      {0.0, 0.0}, {5.0, 0.0}, {-5.0, -5.0}, {1e-12, 1e-12}};
  for (const Eigen::Vector2d& goal : goals) {
    const dwa::Trajectory trajectory =
        planner.rollout(poseAt(0, 0, 0.5), dwa::Control(0.0, 0.0));
    const double cost = planner.goalHeadingCost(trajectory, goal);
    EXPECT_TRUE(std::isfinite(cost)) << "goal " << goal.transpose();
    EXPECT_GE(cost, 0.0);
    EXPECT_LE(cost, dwa::kPi + 1e-9);
  }
}

// --- Clearance cost ---

TEST(ClearanceCost, IsInfiniteOnCollision) {
  dwa::PlannerConfig config;
  config.robotRadius = 1.0;
  const dwa::DwaPlanner planner = makePlanner({{2.0, 0.0}}, config);

  const dwa::Trajectory trajectory = planner.rollout(poseAt(0, 0, 0), dwa::Control(1.0, 0.0));
  EXPECT_FALSE(std::isfinite(planner.clearanceCost(trajectory)));
  EXPECT_TRUE(planner.collides(trajectory));
}

TEST(ClearanceCost, IsZeroWithNoObstacles) {
  const dwa::DwaPlanner planner = makePlanner();
  const dwa::Trajectory trajectory = planner.rollout(poseAt(0, 0, 0), dwa::Control(1.0, 0.0));
  EXPECT_DOUBLE_EQ(planner.clearanceCost(trajectory), 0.0);
}

TEST(ClearanceCost, GrowsAsObstaclesGetCloser) {
  dwa::PlannerConfig config;
  config.robotRadius = 0.1;
  const dwa::DwaPlanner near = makePlanner({{2.0, 0.5}}, config);
  const dwa::DwaPlanner far = makePlanner({{2.0, 5.0}}, config);

  const dwa::Trajectory trajectory = near.rollout(poseAt(0, 0, 0), dwa::Control(1.0, 0.0));
  EXPECT_GT(near.clearanceCost(trajectory), far.clearanceCost(trajectory));
}

// Regression: an earlier version stepped the trajectory by 2, so an obstacle that
// only ever lined up with an odd-indexed pose was invisible to it.
TEST(ClearanceCost, DetectsObstacleOnAnOddIndexedPose) {
  dwa::PlannerConfig config;
  config.dt = 1.0;
  config.predictionHorizon = 4.0;
  config.robotRadius = 0.2;
  // v = 1 m/s with dt = 1 s puts poses at x = 0,1,2,3,4; index 1 is x = 1.
  const dwa::DwaPlanner planner = makePlanner({{1.0, 0.0}}, config);

  const dwa::Trajectory trajectory = planner.rollout(poseAt(0, 0, 0), dwa::Control(1.0, 0.0));
  EXPECT_TRUE(planner.collides(trajectory));
  EXPECT_FALSE(std::isfinite(planner.clearanceCost(trajectory)));
}

TEST(ClearanceCost, RespectsObstacleInflation) {
  dwa::PlannerConfig config;
  config.robotRadius = 0.5;
  config.obstacleRadius = 0.0;
  const dwa::DwaPlanner thin = makePlanner({{2.0, 0.8}}, config);

  config.obstacleRadius = 1.0;
  const dwa::DwaPlanner inflated = makePlanner({{2.0, 0.8}}, config);

  const dwa::Trajectory trajectory = thin.rollout(poseAt(0, 0, 0), dwa::Control(1.0, 0.0));
  EXPECT_FALSE(thin.collides(trajectory));
  EXPECT_TRUE(inflated.collides(trajectory));
}

// --- Velocity cost ---

TEST(VelocityCost, RewardsFasterTrajectories) {
  dwa::PlannerConfig config;
  config.maxLinearVelocity = 1.0;
  const dwa::DwaPlanner planner = makePlanner({}, config);

  const dwa::Trajectory fast = planner.rollout(poseAt(0, 0, 0), dwa::Control(1.0, 0.0));
  const dwa::Trajectory slow = planner.rollout(poseAt(0, 0, 0), dwa::Control(0.2, 0.0));

  EXPECT_LT(planner.velocityCost(fast), planner.velocityCost(slow));
  EXPECT_NEAR(planner.velocityCost(fast), 0.0, 1e-12);
}

TEST(VelocityCost, IsNeverNegative) {
  const dwa::DwaPlanner planner = makePlanner();
  for (double v = -0.5; v <= 1.0; v += 0.1) {
    const dwa::Trajectory trajectory = planner.rollout(poseAt(0, 0, 0), dwa::Control(v, 0.0));
    EXPECT_GE(planner.velocityCost(trajectory), 0.0);
  }
}

// --- plan() and recovery ---

TEST(Plan, FindsATrajectoryInOpenSpace) {
  const dwa::DwaPlanner planner = makePlanner();
  const std::optional<dwa::PlanResult> result =
      planner.plan(poseAt(0, 0, 0), Eigen::Vector2d(10.0, 0.0));

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->trajectory.empty());
  EXPECT_TRUE(std::isfinite(result->control(0)));
  EXPECT_TRUE(std::isfinite(result->control(1)));
}

TEST(Plan, ChosenTrajectoryIsAlwaysCollisionFree) {
  const std::optional<dwa::Scenario> scenario = dwa::makeScenario("default");
  ASSERT_TRUE(scenario.has_value());
  const dwa::DwaPlanner planner = makePlanner(scenario->obstacles);

  dwa::State state = scenario->start;
  for (int i = 0; i < 300 && !planner.atGoal(state, scenario->goal); ++i) {
    const std::optional<dwa::PlanResult> result = planner.plan(state, scenario->goal);
    if (!result) {
      break;
    }
    EXPECT_FALSE(planner.collides(result->trajectory)) << "at step " << i;
    dwa::DwaPlanner::step(result->control, state, planner.config().dt);
  }
}

// Regression: when every rollout collided, the old code produced an empty
// trajectory and immediately read element [1] of it.
TEST(Plan, ReturnsNulloptWhenEverythingCollides) {
  const std::optional<dwa::Scenario> scenario = dwa::makeScenario("trapped");
  ASSERT_TRUE(scenario.has_value());
  const dwa::DwaPlanner planner = makePlanner(scenario->obstacles);

  EXPECT_FALSE(planner.plan(scenario->start, scenario->goal).has_value());
}

TEST(RecoveryControl, StopsAndRotatesWithinTheWindow) {
  const dwa::DwaPlanner planner = makePlanner();
  const dwa::State state = poseAt(0, 0, 0, 0.8, 0.0);

  const dwa::Control control = planner.recoveryControl(state);
  const dwa::DynamicWindowBounds window = planner.dynamicWindow(state);

  // Decelerating, not jumping straight to zero.
  EXPECT_LT(control(0), state(dwa::kV));
  EXPECT_GE(control(0), window.vMin);
  EXPECT_LE(control(0), window.vMax);
  EXPECT_GE(control(1), window.wMin);
  EXPECT_LE(control(1), window.wMax);
  EXPECT_NE(control(1), 0.0);  // actually rotating
}

TEST(RecoveryControl, ReachesZeroVelocityEventually) {
  const dwa::DwaPlanner planner = makePlanner();
  dwa::State state = poseAt(0, 0, 0, 1.0, 0.0);

  for (int i = 0; i < 200; ++i) {
    const dwa::Control control = planner.recoveryControl(state);
    dwa::DwaPlanner::step(control, state, planner.config().dt);
  }
  EXPECT_NEAR(state(dwa::kV), 0.0, 1e-9);
}

// --- End-to-end behaviour on the scenarios ---

TEST(Scenarios, DefaultReachesTheGoal) {
  const std::optional<dwa::Scenario> scenario = dwa::makeScenario("default");
  ASSERT_TRUE(scenario.has_value());
  const dwa::DwaPlanner planner = makePlanner(scenario->obstacles);

  dwa::State state = scenario->start;
  bool reached = false;
  for (int i = 0; i < 2000; ++i) {
    if (planner.atGoal(state, scenario->goal)) {
      reached = true;
      break;
    }
    const std::optional<dwa::PlanResult> result = planner.plan(state, scenario->goal);
    ASSERT_TRUE(result.has_value()) << "stalled at step " << i;
    dwa::DwaPlanner::step(result->control, state, planner.config().dt);
  }
  EXPECT_TRUE(reached);
}

// The old origin-referenced heading cost only behaved when the start and goal
// straddled the world origin.
TEST(Scenarios, OffsetFromOriginAlsoReachesTheGoal) {
  const std::optional<dwa::Scenario> scenario = dwa::makeScenario("offset");
  ASSERT_TRUE(scenario.has_value());
  const dwa::DwaPlanner planner = makePlanner(scenario->obstacles);

  dwa::State state = scenario->start;
  bool reached = false;
  for (int i = 0; i < 2000; ++i) {
    if (planner.atGoal(state, scenario->goal)) {
      reached = true;
      break;
    }
    const std::optional<dwa::PlanResult> result = planner.plan(state, scenario->goal);
    ASSERT_TRUE(result.has_value()) << "stalled at step " << i;
    dwa::DwaPlanner::step(result->control, state, planner.config().dt);
  }
  EXPECT_TRUE(reached);
}

// The corridor is sealed, so the far side is unreachable. Note this needs the
// *side* walls too: a plain finite wall can be driven around given enough
// time, which is correct behaviour rather than a tunneling failure.
TEST(Scenarios, SealedCorridorIsNeverCrossed) {
  const std::optional<dwa::Scenario> scenario = dwa::makeScenario("corridor");
  ASSERT_TRUE(scenario.has_value());
  const dwa::DwaPlanner planner = makePlanner(scenario->obstacles);

  dwa::State state = scenario->start;
  ASSERT_LT(state(dwa::kY), 0.0);

  for (int i = 0; i < 600; ++i) {
    const std::optional<dwa::PlanResult> result = planner.plan(state, scenario->goal);
    const dwa::Control control =
        result ? result->control : planner.recoveryControl(state);
    dwa::DwaPlanner::step(control, state, planner.config().dt);

    ASSERT_LT(state(dwa::kY), 0.0) << "crossed the sealed wall at step " << i;
  }
}

// The invariant that actually matters, and the one the every-other-pose
// subsampling bug broke: the executed path must never enter an obstacle.
TEST(Scenarios, ExecutedPathNeverEntersAnObstacle) {
  for (const char* name : {"default", "offset", "corridor"}) {
    const std::optional<dwa::Scenario> scenario = dwa::makeScenario(name);
    ASSERT_TRUE(scenario.has_value()) << name;
    const dwa::DwaPlanner planner = makePlanner(scenario->obstacles);
    const double contact = planner.config().robotRadius + planner.config().obstacleRadius;

    dwa::State state = scenario->start;
    for (int i = 0; i < 600 && !planner.atGoal(state, scenario->goal); ++i) {
      const std::optional<dwa::PlanResult> result = planner.plan(state, scenario->goal);
      const dwa::Control control =
          result ? result->control : planner.recoveryControl(state);
      dwa::DwaPlanner::step(control, state, planner.config().dt);

      for (const dwa::Obstacle& obstacle : scenario->obstacles) {
        const double distance =
            std::hypot(state(dwa::kX) - obstacle.x, state(dwa::kY) - obstacle.y);
        ASSERT_GT(distance, contact - 1e-9)
            << name << ": entered obstacle at step " << i;
      }
    }
  }
}

TEST(Scenarios, TrappedNeverCrashesAndKeepsMoving) {
  const std::optional<dwa::Scenario> scenario = dwa::makeScenario("trapped");
  ASSERT_TRUE(scenario.has_value());
  const dwa::DwaPlanner planner = makePlanner(scenario->obstacles);

  dwa::State state = scenario->start;
  int recoveries = 0;
  for (int i = 0; i < 200; ++i) {
    const std::optional<dwa::PlanResult> result = planner.plan(state, scenario->goal);
    dwa::Control control;
    if (result) {
      control = result->control;
    } else {
      control = planner.recoveryControl(state);
      ++recoveries;
    }
    dwa::DwaPlanner::step(control, state, planner.config().dt);
    ASSERT_TRUE(std::isfinite(state(dwa::kX)));
    ASSERT_TRUE(std::isfinite(state(dwa::kY)));
  }
  EXPECT_GT(recoveries, 0);
}

}  // namespace
