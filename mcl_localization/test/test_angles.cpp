#include "mcl/angles.hpp"

#include <vector>

#include <gtest/gtest.h>

#include "mcl/types.hpp"

namespace {

constexpr double kTol = 1e-9;

TEST(WrapToPi, LeavesInRangeAnglesAlone) {
  EXPECT_NEAR(mcl::wrapToPi(0.0), 0.0, kTol);
  EXPECT_NEAR(mcl::wrapToPi(1.0), 1.0, kTol);
  EXPECT_NEAR(mcl::wrapToPi(-1.0), -1.0, kTol);
}

// Regression: fromPiToMinusPi used a single if/else-if, so it removed at most
// one revolution -- 5*pi came back as 3*pi, still out of range.
TEST(WrapToPi, WrapsMultipleRevolutions) {
  EXPECT_NEAR(std::abs(mcl::wrapToPi(5.0 * mcl::kPi)), mcl::kPi, 1e-9);
  EXPECT_NEAR(std::abs(mcl::wrapToPi(-5.0 * mcl::kPi)), mcl::kPi, 1e-9);
  EXPECT_NEAR(mcl::wrapToPi(100.0 * mcl::kPi), 0.0, 1e-7);
}

TEST(WrapToPi, AlwaysReturnsInRange) {
  for (int i = -500; i <= 500; ++i) {
    const double angle = 0.41 * static_cast<double>(i);
    const double wrapped = mcl::wrapToPi(angle);
    EXPECT_LE(wrapped, mcl::kPi + kTol) << "angle " << angle;
    EXPECT_GE(wrapped, -mcl::kPi - kTol) << "angle " << angle;
  }
}

TEST(AngleDiff, TakesTheShortWayAcrossTheBranchCut) {
  EXPECT_NEAR(mcl::angleDiff(-mcl::kPi + 0.1, mcl::kPi - 0.1), 0.2, 1e-9);
  EXPECT_NEAR(mcl::angleDiff(mcl::kPi - 0.1, -mcl::kPi + 0.1), -0.2, 1e-9);
}

TEST(AngleDiff, IsZeroForAnglesDifferingByFullRevolutions) {
  EXPECT_NEAR(mcl::angleDiff(0.7 + 2.0 * mcl::kPi, 0.7), 0.0, 1e-9);
  EXPECT_NEAR(mcl::angleDiff(0.7 - 4.0 * mcl::kPi, 0.7), 0.0, 1e-9);
}

// --- circularMean ---

double meanOf(const std::vector<double>& angles) {
  std::vector<mcl::Particle> particles;
  for (double angle : angles) {
    mcl::Particle particle;
    particle.pose.theta = angle;
    particle.weight = 1.0 / static_cast<double>(angles.size());
    particles.push_back(particle);
  }
  return mcl::circularMean(
      particles.begin(), particles.end(),
      [](const mcl::Particle& p) { return p.pose.theta; },
      [](const mcl::Particle& p) { return p.weight; });
}

TEST(CircularMean, MatchesTheArithmeticMeanAwayFromTheBranchCut) {
  EXPECT_NEAR(meanOf({0.1, 0.2, 0.3}), 0.2, 1e-9);
}

// Regression: an earlier version averaged yaw arithmetically. The arithmetic mean
// of +3.1 and -3.1 is 0 -- pointing the opposite way from both inputs.
TEST(CircularMean, HandlesTheBranchCut) {
  const double mean = meanOf({3.1, -3.1});
  EXPECT_NEAR(std::abs(mean), mcl::kPi, 1e-6);
  EXPECT_GT(std::abs(mean), 3.0) << "arithmetic mean would have given ~0";
}

TEST(CircularMean, IsWeighted) {
  std::vector<mcl::Particle> particles(2);
  particles[0].pose.theta = 0.0;
  particles[0].weight = 0.99;
  particles[1].pose.theta = 1.0;
  particles[1].weight = 0.01;

  const double mean = mcl::circularMean(
      particles.begin(), particles.end(),
      [](const mcl::Particle& p) { return p.pose.theta; },
      [](const mcl::Particle& p) { return p.weight; });
  EXPECT_LT(mean, 0.05) << "the heavy particle must dominate";
}

TEST(CircularMean, ReturnsZeroForAZeroResultant) {
  // Perfectly opposed angles have no mean direction; must not be NaN.
  const double mean = meanOf({0.0, mcl::kPi, 0.5 * mcl::kPi, -0.5 * mcl::kPi});
  EXPECT_TRUE(std::isfinite(mean));
}

// --- compose: the sensor-offset transform ---

// Regression: applying the offset as
//   sx = x + ox*cos(theta);  sy = y + oy*sin(theta);
// dropping oy from x and ox from y entirely.
TEST(Compose, IsAFullRigidTransform) {
  // Facing +y, a purely forward offset must move along +y, not +x.
  const mcl::Pose2D base(0.0, 0.0, 0.5 * mcl::kPi);
  const mcl::Pose2D result = mcl::compose(base, mcl::Pose2D(1.0, 0.0, 0.0));
  EXPECT_NEAR(result.x, 0.0, 1e-9);
  EXPECT_NEAR(result.y, 1.0, 1e-9);
}

TEST(Compose, LateralOffsetAffectsBothAxes) {
  const mcl::Pose2D base(0.0, 0.0, 0.0);
  const mcl::Pose2D result = mcl::compose(base, mcl::Pose2D(0.0, 1.0, 0.0));
  EXPECT_NEAR(result.x, 0.0, 1e-9);
  EXPECT_NEAR(result.y, 1.0, 1e-9);

  // Rotated 90 degrees, that same lateral offset must point along -x.
  const mcl::Pose2D rotated =
      mcl::compose(mcl::Pose2D(0.0, 0.0, 0.5 * mcl::kPi), mcl::Pose2D(0.0, 1.0, 0.0));
  EXPECT_NEAR(rotated.x, -1.0, 1e-9);
  EXPECT_NEAR(rotated.y, 0.0, 1e-9);
}

TEST(Compose, PreservesDistanceFromTheBase) {
  const mcl::Pose2D offset(0.75, -0.23, 0.0);
  const double expected = std::hypot(offset.x, offset.y);
  for (double theta = -mcl::kPi; theta < mcl::kPi; theta += 0.37) {
    const mcl::Pose2D base(3.0, -2.0, theta);
    const mcl::Pose2D result = mcl::compose(base, offset);
    EXPECT_NEAR(std::hypot(result.x - base.x, result.y - base.y), expected, 1e-9);
  }
}

TEST(Compose, WrapsTheResultingYaw) {
  const mcl::Pose2D result =
      mcl::compose(mcl::Pose2D(0.0, 0.0, 3.0), mcl::Pose2D(0.0, 0.0, 3.0));
  EXPECT_LE(result.theta, mcl::kPi + kTol);
  EXPECT_GE(result.theta, -mcl::kPi - kTol);
}

}  // namespace
