#include "mcl/motion_model.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

namespace {

mcl::MotionModel::Params noisyParams() {
  mcl::MotionModel::Params params;
  params.alpha1 = 0.1;
  params.alpha2 = 0.1;
  params.alpha3 = 0.1;
  params.alpha4 = 0.1;
  return params;
}

mcl::MotionModel::Params noiselessParams() {
  mcl::MotionModel::Params params;
  params.alpha1 = 0.0;
  params.alpha2 = 0.0;
  params.alpha3 = 0.0;
  params.alpha4 = 0.0;
  return params;
}

// Regression: an earlier version omitted the absolute values the algorithm
// requires, so a negative rotation produced a NEGATIVE standard deviation --
// which then sign-flipped the sample drawn from it -- and in the translation
// term two opposite rotations cancelled instead of accumulating noise.
TEST(MotionModel, DeviationsAreNeverNegative) {
  mcl::MotionModel model(noisyParams(), 1);

  const std::vector<double> values = {-3.0, -1.0, -0.1, 0.0, 0.1, 1.0, 3.0};
  for (double rotation1 : values) {
    for (double rotation2 : values) {
      for (double translation : {0.0, 0.5, 2.0}) {
        const mcl::MotionModel::Deviations sigma =
            model.deviations(rotation1, translation, rotation2);
        EXPECT_GE(sigma.rotation1, 0.0)
            << rotation1 << " " << translation << " " << rotation2;
        EXPECT_GE(sigma.translation, 0.0)
            << rotation1 << " " << translation << " " << rotation2;
        EXPECT_GE(sigma.rotation2, 0.0)
            << rotation1 << " " << translation << " " << rotation2;
      }
    }
  }
}

TEST(MotionModel, OppositeRotationsDoNotCancelInTheTranslationSigma) {
  mcl::MotionModel model(noisyParams(), 1);
  // +1 and -1 rad must accumulate noise, not sum to zero.
  const mcl::MotionModel::Deviations sigma = model.deviations(1.0, 0.0, -1.0);
  EXPECT_GT(sigma.translation, 0.1);
}

TEST(MotionModel, DeviationsGrowWithTheDeltaMagnitude) {
  mcl::MotionModel model(noisyParams(), 1);
  const mcl::MotionModel::Deviations small = model.deviations(0.1, 0.1, 0.1);
  const mcl::MotionModel::Deviations large = model.deviations(1.0, 1.0, 1.0);
  EXPECT_GT(large.rotation1, small.rotation1);
  EXPECT_GT(large.translation, small.translation);
  EXPECT_GT(large.rotation2, small.rotation2);
}

TEST(MotionModel, NoiselessStraightMotionReproducesTheDeltaExactly) {
  mcl::MotionModel model(noiselessParams(), 1);
  const mcl::Pose2D result =
      model.sample(mcl::Pose2D(0.0, 0.0, 0.0), mcl::Pose2D(1.0, 0.0, 0.0),
                   mcl::Pose2D(5.0, 5.0, 0.0));
  EXPECT_NEAR(result.x, 6.0, 1e-9);
  EXPECT_NEAR(result.y, 5.0, 1e-9);
  EXPECT_NEAR(result.theta, 0.0, 1e-9);
}

TEST(MotionModel, AppliesTheDeltaInTheParticleFrameNotTheOdomFrame) {
  mcl::MotionModel model(noiselessParams(), 1);
  // Odometry moves 1 m along its own +x; the particle faces +y, so it must
  // move along world +y.
  const mcl::Pose2D result =
      model.sample(mcl::Pose2D(0.0, 0.0, 0.0), mcl::Pose2D(1.0, 0.0, 0.0),
                   mcl::Pose2D(0.0, 0.0, 0.5 * mcl::kPi));
  EXPECT_NEAR(result.x, 0.0, 1e-9);
  EXPECT_NEAR(result.y, 1.0, 1e-9);
}

TEST(MotionModel, ZeroOdometryDeltaLeavesTheParticleAlone) {
  mcl::MotionModel model(noisyParams(), 1);
  const mcl::Pose2D start(2.0, 3.0, 0.4);
  for (int i = 0; i < 50; ++i) {
    const mcl::Pose2D result =
        model.sample(mcl::Pose2D(1.0, 1.0, 0.2), mcl::Pose2D(1.0, 1.0, 0.2), start);
    EXPECT_NEAR(result.x, start.x, 1e-12);
    EXPECT_NEAR(result.y, start.y, 1e-12);
    EXPECT_NEAR(result.theta, start.theta, 1e-12);
  }
}

// Regression: with the rotation deltas unwrapped, a small physical rotation
// straddling +-pi came out near 2*pi, which multiplied into an enormous
// standard deviation and scattered the cloud.
TEST(MotionModel, SmallRotationAcrossTheBranchCutStaysSmall) {
  mcl::MotionModel model(noisyParams(), 1);

  // 0.2 rad of rotation, expressed as +pi-0.1 -> -pi+0.1.
  const mcl::Pose2D previous(0.0, 0.0, mcl::kPi - 0.1);
  const mcl::Pose2D current(0.0, 0.0, -mcl::kPi + 0.1);

  const mcl::MotionModel::Deviations sigma =
      model.deviations(0.0, 0.0, mcl::angleDiff(current.theta, previous.theta));
  EXPECT_LT(sigma.rotation2, 0.1) << "an unwrapped delta would give ~0.6 here";

  for (int i = 0; i < 100; ++i) {
    const mcl::Pose2D result = model.sample(previous, current, mcl::Pose2D(0, 0, 0));
    EXPECT_LT(std::abs(result.theta), 1.0) << "cloud should not explode";
  }
}

TEST(MotionModel, KeepsYawWrappedOverManySteps) {
  mcl::MotionModel model(noisyParams(), 7);
  mcl::Pose2D particle(0.0, 0.0, 0.0);
  mcl::Pose2D previous(0.0, 0.0, 0.0);

  for (int i = 0; i < 400; ++i) {
    // Keep rotating in the same direction; yaw must not grow without bound.
    const mcl::Pose2D current(0.0, 0.0, mcl::wrapToPi(previous.theta + 0.3));
    particle = model.sample(previous, current, particle);
    previous = current;
    ASSERT_LE(particle.theta, mcl::kPi + 1e-9) << "step " << i;
    ASSERT_GE(particle.theta, -mcl::kPi - 1e-9) << "step " << i;
  }
}

// Regression: with translation ~0, atan2 of pure odometry noise yields an
// essentially random heading, injecting spurious rotation while stationary.
TEST(MotionModel, NearStationaryMotionDoesNotInjectSpuriousRotation) {
  mcl::MotionModel::Params params = noiselessParams();
  params.minTranslation = 1e-3;
  mcl::MotionModel model(params, 3);

  // 1e-6 m of lateral jitter, no real rotation.
  const mcl::Pose2D result =
      model.sample(mcl::Pose2D(0.0, 0.0, 0.0), mcl::Pose2D(0.0, 1e-6, 0.0),
                   mcl::Pose2D(0.0, 0.0, 0.0));
  EXPECT_NEAR(result.theta, 0.0, 1e-9)
      << "a sub-millimetre jitter must not rotate the particle 90 degrees";
}

TEST(MotionModel, IsReproducibleForAGivenSeed) {
  mcl::MotionModel a(noisyParams(), 1234);
  mcl::MotionModel b(noisyParams(), 1234);
  const mcl::Pose2D previous(0.0, 0.0, 0.0);
  const mcl::Pose2D current(0.5, 0.2, 0.1);

  for (int i = 0; i < 20; ++i) {
    const mcl::Pose2D pa = a.sample(previous, current, mcl::Pose2D(1, 1, 0));
    const mcl::Pose2D pb = b.sample(previous, current, mcl::Pose2D(1, 1, 0));
    EXPECT_DOUBLE_EQ(pa.x, pb.x);
    EXPECT_DOUBLE_EQ(pa.y, pb.y);
    EXPECT_DOUBLE_EQ(pa.theta, pb.theta);
  }
}

TEST(MotionModel, DifferentSeedsGiveDifferentSamples) {
  mcl::MotionModel a(noisyParams(), 1);
  mcl::MotionModel b(noisyParams(), 2);
  const mcl::Pose2D previous(0.0, 0.0, 0.0);
  const mcl::Pose2D current(1.0, 0.0, 0.0);

  const mcl::Pose2D pa = a.sample(previous, current, mcl::Pose2D(0, 0, 0));
  const mcl::Pose2D pb = b.sample(previous, current, mcl::Pose2D(0, 0, 0));
  EXPECT_NE(pa.x, pb.x);
}

TEST(MotionModel, NoiseIsCentredOnTheNominalDelta) {
  mcl::MotionModel model(noisyParams(), 42);
  const mcl::Pose2D previous(0.0, 0.0, 0.0);
  const mcl::Pose2D current(1.0, 0.0, 0.0);

  double sumX = 0.0;
  const int samples = 4000;
  for (int i = 0; i < samples; ++i) {
    sumX += model.sample(previous, current, mcl::Pose2D(0, 0, 0)).x;
  }
  EXPECT_NEAR(sumX / samples, 1.0, 0.02);
}

}  // namespace
