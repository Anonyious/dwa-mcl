#include "mcl/particle_filter.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

#include "test_helpers.hpp"

namespace {

std::vector<mcl::Particle> makeParticles(const std::vector<double>& weights) {
  std::vector<mcl::Particle> particles;
  for (std::size_t i = 0; i < weights.size(); ++i) {
    mcl::Particle particle;
    particle.pose = mcl::Pose2D(static_cast<double>(i), 0.0, 0.0);
    particle.weight = weights[i];
    particles.push_back(particle);
  }
  return particles;
}

// ------------------------------------------------------------ resampler

TEST(LowVarianceResample, PreservesTheParticleCount) {
  const std::vector<mcl::Particle> source = makeParticles({0.25, 0.25, 0.25, 0.25});
  std::vector<mcl::Particle> out;
  mcl::lowVarianceResample(source, out, 0.1);
  EXPECT_EQ(out.size(), source.size());
}

TEST(LowVarianceResample, ResetsWeightsToUniform) {
  const std::vector<mcl::Particle> source = makeParticles({0.7, 0.1, 0.1, 0.1});
  std::vector<mcl::Particle> out;
  mcl::lowVarianceResample(source, out, 0.0);

  // Regression: the line resetting the weights was commented out, leaving
  // the pre-resample weights on the new cloud.
  for (const mcl::Particle& particle : out) {
    EXPECT_NEAR(particle.weight, 0.25, 1e-12);
  }
}

TEST(LowVarianceResample, ConcentratesOnHeavyParticles) {
  // Particle 2 holds almost all the weight.
  const std::vector<mcl::Particle> source =
      makeParticles({0.01, 0.01, 0.97, 0.01});
  std::vector<mcl::Particle> out;
  mcl::lowVarianceResample(source, out, 0.0);

  int fromTwo = 0;
  for (const mcl::Particle& particle : out) {
    if (std::abs(particle.pose.x - 2.0) < 1e-12) {
      ++fromTwo;
    }
  }
  EXPECT_GE(fromTwo, 3);
}

TEST(LowVarianceResample, UniformWeightsReproduceEveryParticleOnce) {
  const std::size_t n = 50;
  const std::vector<mcl::Particle> source =
      makeParticles(std::vector<double>(n, 1.0 / static_cast<double>(n)));
  std::vector<mcl::Particle> out;
  mcl::lowVarianceResample(source, out, 0.5 / static_cast<double>(n));

  std::vector<int> seen(n, 0);
  for (const mcl::Particle& particle : out) {
    const int index = static_cast<int>(std::lround(particle.pose.x));
    ASSERT_GE(index, 0);
    ASSERT_LT(index, static_cast<int>(n));
    ++seen[static_cast<std::size_t>(index)];
  }
  for (std::size_t i = 0; i < n; ++i) {
    EXPECT_EQ(seen[i], 1) << "particle " << i;
  }
}

// Regression: the inner loop guarded on the OUTER index, which never changed
// inside it, so the advancing index could walk past the end of the vector.
// Weights that sum to slightly under 1 make the final step overshoot, which
// is exactly when that happened.
TEST(LowVarianceResample, StaysInBoundsWhenWeightsSumBelowOne) {
  const std::size_t n = 32;
  // Deliberately sums to ~0.99, as floating-point normalization can.
  std::vector<double> weights(n, 0.99 / static_cast<double>(n));
  const std::vector<mcl::Particle> source = makeParticles(weights);

  std::vector<mcl::Particle> out;
  // The largest legal start offset is the worst case for overshoot.
  mcl::lowVarianceResample(source, out, 1.0 / static_cast<double>(n) - 1e-15);

  ASSERT_EQ(out.size(), n);
  for (const mcl::Particle& particle : out) {
    // Every output must be one of the inputs, i.e. x in [0, n).
    EXPECT_GE(particle.pose.x, 0.0);
    EXPECT_LT(particle.pose.x, static_cast<double>(n));
    EXPECT_TRUE(std::isfinite(particle.pose.x));
  }
}

TEST(LowVarianceResample, StaysInBoundsWithAllWeightOnTheFirstParticle) {
  std::vector<double> weights(16, 0.0);
  weights[0] = 1.0;
  const std::vector<mcl::Particle> source = makeParticles(weights);

  std::vector<mcl::Particle> out;
  mcl::lowVarianceResample(source, out, 1.0 / 16.0 - 1e-15);
  ASSERT_EQ(out.size(), 16u);
  for (const mcl::Particle& particle : out) {
    EXPECT_NEAR(particle.pose.x, 0.0, 1e-12);
  }
}

TEST(LowVarianceResample, StaysInBoundsWithAllWeightOnTheLastParticle) {
  std::vector<double> weights(16, 0.0);
  weights[15] = 1.0;
  const std::vector<mcl::Particle> source = makeParticles(weights);

  std::vector<mcl::Particle> out;
  // startOffset = 0 is the worst case: the algorithm compares `target > c`,
  // so at m = 0 it sees 0 > 0, does not advance, and emits particle 0 even
  // though its weight is zero. That is the textbook algorithm's behaviour
  // (Probabilistic Robotics Table 4.4) and not something this test should
  // pretend otherwise about -- it costs at most one sample out of n, and
  // only when the draw is exactly 0.
  mcl::lowVarianceResample(source, out, 0.0);
  ASSERT_EQ(out.size(), 16u);

  int fromLast = 0;
  for (const mcl::Particle& particle : out) {
    // The property that matters, and the one the index-bound bug broke:
    // every output is a real index into the source.
    ASSERT_GE(particle.pose.x, 0.0);
    ASSERT_LT(particle.pose.x, 16.0);
    if (std::abs(particle.pose.x - 15.0) < 1e-12) {
      ++fromLast;
    }
  }
  EXPECT_GE(fromLast, 15) << "the heavy particle must take all but the m=0 slot";
}

// With any strictly positive offset the m = 0 comparison does advance, so the
// heavy particle takes every slot.
TEST(LowVarianceResample, PositiveOffsetGivesTheLastParticleEverySlot) {
  std::vector<double> weights(16, 0.0);
  weights[15] = 1.0;
  const std::vector<mcl::Particle> source = makeParticles(weights);

  std::vector<mcl::Particle> out;
  mcl::lowVarianceResample(source, out, 1.0 / 32.0);
  ASSERT_EQ(out.size(), 16u);
  for (const mcl::Particle& particle : out) {
    EXPECT_NEAR(particle.pose.x, 15.0, 1e-12);
  }
}

TEST(LowVarianceResample, HandlesAllZeroWeightsWithoutCrashing) {
  const std::vector<mcl::Particle> source = makeParticles(std::vector<double>(8, 0.0));
  std::vector<mcl::Particle> out;
  mcl::lowVarianceResample(source, out, 0.0);
  ASSERT_EQ(out.size(), 8u);
  for (const mcl::Particle& particle : out) {
    EXPECT_TRUE(std::isfinite(particle.pose.x));
  }
}

TEST(LowVarianceResample, HandlesAnEmptyInput) {
  std::vector<mcl::Particle> out;
  mcl::lowVarianceResample({}, out, 0.0);
  EXPECT_TRUE(out.empty());
}

// ------------------------------------------------------------ filter setup

class FilterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    map_ = std::make_shared<mcl::OccupancyGrid>(mcl::testing::makeRoom(80, 60, 0.1));

    filterParams_.particleCount = 300;
    filterParams_.seed = 7;
    motionParams_.alpha1 = 0.05;
    motionParams_.alpha2 = 0.05;
    motionParams_.alpha3 = 0.05;
    motionParams_.alpha4 = 0.05;
    sensorParams_.sigmaHit = 0.2;
    sensorParams_.beamSkip = 6;
  }

  std::unique_ptr<mcl::ParticleFilter> makeFilter() {
    auto filter = std::make_unique<mcl::ParticleFilter>(filterParams_, motionParams_,
                                                        sensorParams_);
    filter->setMap(map_);
    return filter;
  }

  std::shared_ptr<const mcl::OccupancyGrid> map_;
  mcl::ParticleFilter::Params filterParams_;
  mcl::MotionModel::Params motionParams_;
  mcl::SensorModel::Params sensorParams_;
};

TEST_F(FilterTest, HasNoMapBeforeSetMap) {
  mcl::ParticleFilter filter(filterParams_, motionParams_, sensorParams_);
  EXPECT_FALSE(filter.hasMap());
  EXPECT_FALSE(filter.initialized());
  EXPECT_FALSE(filter.initializeGlobal());
}

TEST_F(FilterTest, InitializesGloballyInsideFreeSpace) {
  auto filter = makeFilter();
  ASSERT_TRUE(filter->initializeGlobal());
  EXPECT_TRUE(filter->initialized());
  EXPECT_EQ(filter->particles().size(), 300u);

  for (const mcl::Particle& particle : filter->particles()) {
    EXPECT_TRUE(map_->isFreeAtWorld(particle.pose.x, particle.pose.y));
    EXPECT_LE(particle.pose.theta, mcl::kPi + 1e-9);
    EXPECT_GE(particle.pose.theta, -mcl::kPi - 1e-9);
  }
}

TEST_F(FilterTest, GlobalInitializationWeightsSumToOne) {
  auto filter = makeFilter();
  ASSERT_TRUE(filter->initializeGlobal());
  double total = 0.0;
  for (const mcl::Particle& particle : filter->particles()) {
    total += particle.weight;
  }
  EXPECT_NEAR(total, 1.0, 1e-9);
}

// Regression: mapCallback appended to the map, the free/occupied lists and
// the particle vector without clearing any of them, so a second (latched)
// map message corrupted the state and then wrote out of bounds.
TEST_F(FilterTest, SetMapTwiceIsIdempotent) {
  auto filter = makeFilter();
  ASSERT_TRUE(filter->initializeGlobal());
  ASSERT_EQ(filter->particles().size(), 300u);

  filter->setMap(map_);
  EXPECT_FALSE(filter->initialized()) << "derived state must be cleared";
  EXPECT_TRUE(filter->particles().empty());

  ASSERT_TRUE(filter->initializeGlobal());
  EXPECT_EQ(filter->particles().size(), 300u) << "must not have doubled";
}

// ------------------------------------------------------------ estimate()

// Regression: publishPose declared `double x, y, yaw;` uninitialized and then
// accumulated into them, so the published pose and TF were undefined.
TEST_F(FilterTest, EstimateIsFiniteAndCentredOnTheSeedPose) {
  auto filter = makeFilter();
  const mcl::Pose2D seed(4.0, 3.0, 0.5);
  ASSERT_TRUE(filter->initializeAtPose(seed, 0.05, 0.02));

  const mcl::PoseEstimate estimate = filter->estimate();
  ASSERT_TRUE(estimate.valid);
  EXPECT_TRUE(std::isfinite(estimate.mean.x));
  EXPECT_TRUE(std::isfinite(estimate.mean.y));
  EXPECT_TRUE(std::isfinite(estimate.mean.theta));
  EXPECT_NEAR(estimate.mean.x, seed.x, 0.05);
  EXPECT_NEAR(estimate.mean.y, seed.y, 0.05);
  EXPECT_NEAR(estimate.mean.theta, seed.theta, 0.05);
}

TEST_F(FilterTest, EstimateYawUsesACircularMean) {
  auto filter = makeFilter();
  // Seeded at +pi with a wide yaw spread, so particles straddle the cut.
  ASSERT_TRUE(filter->initializeAtPose(mcl::Pose2D(4.0, 3.0, mcl::kPi), 0.05, 0.3));

  const mcl::PoseEstimate estimate = filter->estimate();
  ASSERT_TRUE(estimate.valid);
  // An arithmetic mean over wrapped values would land near 0.
  EXPECT_GT(std::abs(estimate.mean.theta), 2.5)
      << "got " << estimate.mean.theta << "; arithmetic mean would give ~0";
}

TEST_F(FilterTest, EstimateCovarianceGrowsWithTheSpread) {
  auto filter = makeFilter();

  ASSERT_TRUE(filter->initializeAtPose(mcl::Pose2D(4.0, 3.0, 0.0), 0.05, 0.01));
  const double tight = filter->estimate().covariance[0];

  ASSERT_TRUE(filter->initializeAtPose(mcl::Pose2D(4.0, 3.0, 0.0), 0.5, 0.01));
  const double loose = filter->estimate().covariance[0];

  EXPECT_GT(loose, tight);
  EXPECT_GT(tight, 0.0);
}

TEST_F(FilterTest, EstimateIsInvalidWithNoParticles) {
  mcl::ParticleFilter filter(filterParams_, motionParams_, sensorParams_);
  EXPECT_FALSE(filter.estimate().valid);
}

TEST_F(FilterTest, EffectiveSampleSizeIsNAfterUniformInitialization) {
  auto filter = makeFilter();
  ASSERT_TRUE(filter->initializeGlobal());
  EXPECT_NEAR(filter->effectiveSampleSize(), 300.0, 1e-6);
}

// ------------------------------------------------------------ update()

TEST_F(FilterTest, UpdateDoesNothingBeforeInitialization) {
  mcl::ParticleFilter filter(filterParams_, motionParams_, sensorParams_);
  EXPECT_FALSE(filter.update(mcl::Pose2D(), mcl::LaserScan()));
}

// Regression: the previous odometry pose lived in zero-initialized function
// statics, so the first update computed its delta from (0,0,0) rather than
// from the first real reading -- one enormous bogus motion at startup.
TEST_F(FilterTest, FirstUpdateOnlyEstablishesTheOdometryReference) {
  auto filter = makeFilter();
  const mcl::Pose2D seed(4.0, 3.0, 0.0);
  ASSERT_TRUE(filter->initializeAtPose(seed, 0.05, 0.02));

  const mcl::LaserScan scan = mcl::testing::simulateScan(*map_, seed, 180, 20.0);

  // Odometry starts far from the origin; the first call must not treat that
  // as a 1000 m motion.
  const mcl::Pose2D farOdom(1000.0, -500.0, 1.0);
  EXPECT_FALSE(filter->update(farOdom, scan)) << "first call just seeds the reference";

  const mcl::PoseEstimate estimate = filter->estimate();
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(estimate.mean.x, seed.x, 0.2) << "cloud must not have been scattered";
  EXPECT_NEAR(estimate.mean.y, seed.y, 0.2);
}

TEST_F(FilterTest, UpdateIsSkippedBelowTheMotionThreshold) {
  auto filter = makeFilter();
  const mcl::Pose2D seed(4.0, 3.0, 0.0);
  ASSERT_TRUE(filter->initializeAtPose(seed, 0.05, 0.02));
  const mcl::LaserScan scan = mcl::testing::simulateScan(*map_, seed, 180, 20.0);

  ASSERT_FALSE(filter->update(mcl::Pose2D(0.0, 0.0, 0.0), scan));
  // 1 mm of motion is below the 2 cm threshold.
  EXPECT_FALSE(filter->update(mcl::Pose2D(0.001, 0.0, 0.0), scan));
  // 10 cm is not.
  EXPECT_TRUE(filter->update(mcl::Pose2D(0.1, 0.0, 0.0), scan));
}

TEST_F(FilterTest, ParticleCountIsConstantAcrossUpdates) {
  auto filter = makeFilter();
  const mcl::Pose2D seed(4.0, 3.0, 0.0);
  ASSERT_TRUE(filter->initializeAtPose(seed, 0.1, 0.05));

  mcl::Pose2D odom(0.0, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) {
    const mcl::Pose2D truth(seed.x + 0.05 * i, seed.y, 0.0);
    const mcl::LaserScan scan = mcl::testing::simulateScan(*map_, truth, 180, 20.0);
    odom.x += 0.05;
    filter->update(odom, scan);
    ASSERT_EQ(filter->particles().size(), 300u) << "step " << i;
  }
}

TEST_F(FilterTest, WeightsStayFiniteAndNormalizedAcrossUpdates) {
  auto filter = makeFilter();
  const mcl::Pose2D seed(4.0, 3.0, 0.0);
  ASSERT_TRUE(filter->initializeAtPose(seed, 0.1, 0.05));

  mcl::Pose2D odom(0.0, 0.0, 0.0);
  for (int i = 0; i < 20; ++i) {
    const mcl::Pose2D truth(seed.x + 0.05 * i, seed.y, 0.0);
    const mcl::LaserScan scan = mcl::testing::simulateScan(*map_, truth, 180, 20.0);
    odom.x += 0.05;
    filter->update(odom, scan);

    double total = 0.0;
    for (const mcl::Particle& particle : filter->particles()) {
      ASSERT_TRUE(std::isfinite(particle.weight)) << "step " << i;
      ASSERT_GE(particle.weight, 0.0);
      total += particle.weight;
    }
    EXPECT_NEAR(total, 1.0, 1e-6) << "step " << i;
  }
}

// The whole point of the fixes: the filter must follow the robot while
// correcting drifting odometry.
TEST_F(FilterTest, TracksTheRobotThroughDriftingOdometry) {
  auto filter = makeFilter();
  const mcl::Pose2D start(2.0, 2.0, 0.0);
  ASSERT_TRUE(filter->initializeAtPose(start, 0.15, 0.05));

  // Drive +x across the room. Odometry over-reports by 8%, so by the end it
  // is off by nearly 0.3 m.
  const double trueStep = 0.1;
  const double odomStep = 0.108;
  mcl::Pose2D odom(0.0, 0.0, 0.0);
  mcl::Pose2D truth = start;

  for (int i = 0; i < 35; ++i) {
    truth.x += trueStep;
    odom.x += odomStep;
    const mcl::LaserScan scan = mcl::testing::simulateScan(*map_, truth, 180, 20.0);
    filter->update(odom, scan);
  }

  const mcl::PoseEstimate estimate = filter->estimate();
  ASSERT_TRUE(estimate.valid);

  const double filterError = std::hypot(estimate.mean.x - truth.x,
                                        estimate.mean.y - truth.y);
  const double rawOdometryError = std::abs((odom.x + start.x) - truth.x);

  EXPECT_LT(filterError, 0.25) << "estimate (" << estimate.mean.x << ", "
                               << estimate.mean.y << ") vs truth (" << truth.x
                               << ", " << truth.y << ")";
  EXPECT_LT(filterError, rawOdometryError)
      << "the filter must beat raw odometry (" << rawOdometryError << " m)";
  EXPECT_EQ(filter->recoveryCount(), 0);
}

TEST_F(FilterTest, ResamplingKeepsTheCloudFromCollapsingImmediately) {
  auto filter = makeFilter();
  const mcl::Pose2D start(3.0, 3.0, 0.0);
  ASSERT_TRUE(filter->initializeAtPose(start, 0.2, 0.1));

  mcl::Pose2D odom(0.0, 0.0, 0.0);
  mcl::Pose2D truth = start;
  for (int i = 0; i < 10; ++i) {
    truth.x += 0.1;
    odom.x += 0.1;
    const mcl::LaserScan scan = mcl::testing::simulateScan(*map_, truth, 180, 20.0);
    filter->update(odom, scan);
  }

  // Some spread must survive: a cloud with zero variance has lost the
  // ability to correct any further error.
  const mcl::PoseEstimate estimate = filter->estimate();
  ASSERT_TRUE(estimate.valid);
  EXPECT_GT(estimate.covariance[0] + estimate.covariance[4], 1e-8);
}

}  // namespace
