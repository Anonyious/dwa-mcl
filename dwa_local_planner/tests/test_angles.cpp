#include "dwa/angles.hpp"

#include <gtest/gtest.h>

namespace {

constexpr double kTol = 1e-9;

TEST(WrapToPi, LeavesInRangeAnglesAlone) {
  EXPECT_NEAR(dwa::wrapToPi(0.0), 0.0, kTol);
  EXPECT_NEAR(dwa::wrapToPi(1.0), 1.0, kTol);
  EXPECT_NEAR(dwa::wrapToPi(-1.0), -1.0, kTol);
}

TEST(WrapToPi, WrapsSingleRevolution) {
  EXPECT_NEAR(dwa::wrapToPi(2.0 * dwa::kPi), 0.0, kTol);
  EXPECT_NEAR(dwa::wrapToPi(-2.0 * dwa::kPi), 0.0, kTol);
  EXPECT_NEAR(dwa::wrapToPi(1.5 * dwa::kPi), -0.5 * dwa::kPi, kTol);
}

// A single if/else-if correction only ever removes one revolution: 5*pi
// comes back as 3*pi, still out of range. atan2(sin, cos) does not have
// that failure mode.
TEST(WrapToPi, WrapsMultipleRevolutions) {
  // +pi and -pi are the same angle and both lie in the closed range, so which
  // one comes back depends on the sign of a near-zero sin(). Compare the
  // magnitude rather than pinning an arbitrary choice of representative.
  EXPECT_NEAR(std::abs(dwa::wrapToPi(5.0 * dwa::kPi)), dwa::kPi, 1e-9);
  EXPECT_NEAR(std::abs(dwa::wrapToPi(-5.0 * dwa::kPi)), dwa::kPi, 1e-9);
  EXPECT_NEAR(dwa::wrapToPi(100.0 * dwa::kPi), 0.0, 1e-7);
}

TEST(WrapToPi, AlwaysReturnsInRange) {
  for (int i = -200; i <= 200; ++i) {
    const double angle = 0.37 * static_cast<double>(i);
    const double wrapped = dwa::wrapToPi(angle);
    EXPECT_LE(wrapped, dwa::kPi + kTol) << "angle = " << angle;
    EXPECT_GE(wrapped, -dwa::kPi - kTol) << "angle = " << angle;
  }
}

TEST(AngleDiff, TakesTheShortWayRound) {
  // From just below +pi to just above -pi is a small positive step, not a
  // nearly-full revolution.
  const double from = dwa::kPi - 0.1;
  const double to = -dwa::kPi + 0.1;
  EXPECT_NEAR(dwa::angleDiff(to, from), 0.2, 1e-9);
  EXPECT_NEAR(dwa::angleDiff(from, to), -0.2, 1e-9);
}

TEST(AngleDiff, IsZeroForEqualAngles) {
  EXPECT_NEAR(dwa::angleDiff(0.7, 0.7), 0.0, kTol);
  EXPECT_NEAR(dwa::angleDiff(0.7 + 2.0 * dwa::kPi, 0.7), 0.0, 1e-9);
}

TEST(DegToRad, ConvertsKnownValues) {
  EXPECT_NEAR(dwa::degToRad(180.0), dwa::kPi, kTol);
  EXPECT_NEAR(dwa::degToRad(90.0), 0.5 * dwa::kPi, kTol);
  EXPECT_NEAR(dwa::degToRad(0.0), 0.0, kTol);
}

}  // namespace
