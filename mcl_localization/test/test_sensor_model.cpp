#include "mcl/sensor_model.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mcl/likelihood_field.hpp"
#include "test_helpers.hpp"

namespace {

struct Fixture {
  mcl::OccupancyGrid map = mcl::testing::makeRoom(60, 40, 0.1);
  mcl::LikelihoodField field{map};
  mcl::Pose2D truth{3.0, 2.0, 0.3};
  mcl::LaserScan scan = mcl::testing::simulateScan(map, truth, 180, 20.0);
};

mcl::SensorModel::Params defaultParams() {
  mcl::SensorModel::Params params;
  params.sigmaHit = 0.2;
  params.beamSkip = 6;
  return params;
}

TEST(SensorModel, ScoresTheTruePoseAboveADisplacedOne) {
  Fixture f;
  const mcl::SensorModel model(defaultParams());

  const double atTruth = model.logLikelihood(f.truth, f.scan, f.map, f.field);
  const double displaced = model.logLikelihood(
      mcl::Pose2D(f.truth.x + 0.8, f.truth.y, f.truth.theta), f.scan, f.map, f.field);

  EXPECT_GT(atTruth, displaced);
}

TEST(SensorModel, ScoreDecreasesMonotonicallyWithDisplacement) {
  Fixture f;
  const mcl::SensorModel model(defaultParams());

  double previous = model.logLikelihood(f.truth, f.scan, f.map, f.field);
  for (double offset : {0.1, 0.3, 0.6, 1.0}) {
    const double score = model.logLikelihood(
        mcl::Pose2D(f.truth.x + offset, f.truth.y, f.truth.theta), f.scan, f.map,
        f.field);
    EXPECT_LT(score, previous) << "offset " << offset;
    previous = score;
  }
}

TEST(SensorModel, ScoresTheTruePoseAboveARotatedOne) {
  Fixture f;
  const mcl::SensorModel model(defaultParams());

  const double atTruth = model.logLikelihood(f.truth, f.scan, f.map, f.field);
  const double rotated = model.logLikelihood(
      mcl::Pose2D(f.truth.x, f.truth.y, f.truth.theta + 0.6), f.scan, f.map, f.field);

  EXPECT_GT(atTruth, rotated);
}

TEST(SensorModel, IsAlwaysFinite) {
  Fixture f;
  const mcl::SensorModel model(defaultParams());

  const std::vector<mcl::Pose2D> poses = {
      f.truth,
      mcl::Pose2D(0.0, 0.0, 0.0),        // on the wall
      mcl::Pose2D(-100.0, -100.0, 0.0),  // far off the map
      mcl::Pose2D(1e9, 1e9, 0.0),        // absurd
  };
  for (const mcl::Pose2D& pose : poses) {
    EXPECT_TRUE(std::isfinite(model.logLikelihood(pose, f.scan, f.map, f.field)));
  }
}

TEST(SensorModel, PenalisesAPoseOutsideFreeSpace) {
  Fixture f;
  const mcl::SensorModel model(defaultParams());

  const double atTruth = model.logLikelihood(f.truth, f.scan, f.map, f.field);
  const double offMap = model.logLikelihood(mcl::Pose2D(-100.0, 0.0, 0.0), f.scan,
                                            f.map, f.field);
  EXPECT_LT(offMap, atTruth);
  EXPECT_NEAR(offMap, defaultParams().invalidPoseLogLikelihood, 1e-9);
}

// Regression: an earlier version looped `for (int i = 0; i < 720; ++i)` over
// scan.ranges, which is an out-of-bounds read on any scan that is not
// exactly 720 beams.
TEST(SensorModel, HandlesScansOfAnyLength) {
  Fixture f;
  const mcl::SensorModel model(defaultParams());

  for (std::size_t beams : {1u, 7u, 60u, 180u, 721u}) {
    const mcl::LaserScan scan =
        mcl::testing::simulateScan(f.map, f.truth, beams, 20.0);
    EXPECT_TRUE(std::isfinite(model.logLikelihood(f.truth, scan, f.map, f.field)))
        << beams << " beams";
  }

  mcl::LaserScan empty;
  empty.rangeMax = 10.0;
  EXPECT_TRUE(std::isfinite(model.logLikelihood(f.truth, empty, f.map, f.field)));
}

TEST(SensorModel, IgnoresNonFiniteAndOutOfRangeReturns) {
  Fixture f;
  const mcl::SensorModel model(defaultParams());

  mcl::LaserScan scan = f.scan;
  const double clean = model.logLikelihood(f.truth, scan, f.map, f.field);

  // A real driver reports NaN or inf for "no echo".
  for (std::size_t i = 0; i < scan.ranges.size(); i += 3) {
    scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
  }
  const double withNaNs = model.logLikelihood(f.truth, scan, f.map, f.field);
  EXPECT_TRUE(std::isfinite(withNaNs));
  // Dropping beams should not change the per-beam mean much.
  EXPECT_NEAR(withNaNs, clean, std::abs(clean) * 0.5 + 1.0);
}

TEST(SensorModel, BeamsUsedFollowsTheSkip) {
  mcl::LaserScan scan;
  scan.ranges.resize(100);

  mcl::SensorModel::Params params;
  params.beamSkip = 1;
  EXPECT_EQ(mcl::SensorModel(params).beamsUsed(scan), 100u);
  params.beamSkip = 10;
  EXPECT_EQ(mcl::SensorModel(params).beamsUsed(scan), 10u);
  params.beamSkip = 0;  // clamped to 1
  EXPECT_EQ(mcl::SensorModel(params).beamsUsed(scan), 100u);
}

// Normalizing by the beam count makes the score a per-beam mean, so beamSkip
// becomes purely a compute knob. Without it the score scales with the number
// of beams, which silently retunes how peaked the weights are.
TEST(SensorModel, NormalizedScoreIsNearlyIndependentOfBeamSkip) {
  Fixture f;
  mcl::SensorModel::Params params = defaultParams();
  params.normalizeByBeamCount = true;

  params.beamSkip = 1;
  const double dense = mcl::SensorModel(params).logLikelihood(f.truth, f.scan, f.map,
                                                              f.field);
  params.beamSkip = 12;
  const double sparse = mcl::SensorModel(params).logLikelihood(f.truth, f.scan, f.map,
                                                               f.field);

  EXPECT_NEAR(dense, sparse, 0.5 * std::abs(dense) + 0.5);
}

TEST(SensorModel, UnnormalizedScoreScalesWithBeamCount) {
  Fixture f;
  mcl::SensorModel::Params params = defaultParams();
  params.normalizeByBeamCount = false;

  params.beamSkip = 1;
  const double dense = mcl::SensorModel(params).logLikelihood(f.truth, f.scan, f.map,
                                                              f.field);
  params.beamSkip = 10;
  const double sparse = mcl::SensorModel(params).logLikelihood(f.truth, f.scan, f.map,
                                                               f.field);

  // ~10x as many beams summed, so roughly 10x the magnitude.
  //
  // Note the score is POSITIVE for a well-matched pose: the Gaussian is a
  // density, and 1/sqrt(2*pi*sigma^2) is about 1.99 at sigma = 0.2 m, so a
  // beam landing on an obstacle has probability above 1 and a positive log.
  // Only the ratio is meaningful here, not the sign.
  ASSERT_GT(std::abs(sparse), 1e-6);
  const double ratio = dense / sparse;
  EXPECT_GT(ratio, 5.0);
  EXPECT_LT(ratio, 20.0);
}

TEST(SensorModel, LargerSigmaIsMoreForgiving) {
  Fixture f;
  const mcl::Pose2D displaced(f.truth.x + 0.4, f.truth.y, f.truth.theta);

  mcl::SensorModel::Params tight = defaultParams();
  tight.sigmaHit = 0.05;
  mcl::SensorModel::Params loose = defaultParams();
  loose.sigmaHit = 0.5;

  const double tightGap =
      mcl::SensorModel(tight).logLikelihood(f.truth, f.scan, f.map, f.field) -
      mcl::SensorModel(tight).logLikelihood(displaced, f.scan, f.map, f.field);
  const double looseGap =
      mcl::SensorModel(loose).logLikelihood(f.truth, f.scan, f.map, f.field) -
      mcl::SensorModel(loose).logLikelihood(displaced, f.scan, f.map, f.field);

  EXPECT_GT(tightGap, looseGap);
}

TEST(SensorModel, EmptyMapScoresZeroRatherThanCrashing) {
  const mcl::SensorModel model(defaultParams());
  const mcl::OccupancyGrid empty;
  const mcl::LikelihoodField emptyField;
  mcl::LaserScan scan;
  scan.ranges.assign(10, 1.0f);
  scan.rangeMax = 10.0;

  EXPECT_DOUBLE_EQ(model.logLikelihood(mcl::Pose2D(), scan, empty, emptyField), 0.0);
}

}  // namespace
