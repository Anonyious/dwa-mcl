#ifndef MCL_PARTICLE_FILTER_HPP_
#define MCL_PARTICLE_FILTER_HPP_

#include <cstdint>
#include <memory>
#include <random>
#include <vector>

#include "mcl/likelihood_field.hpp"
#include "mcl/motion_model.hpp"
#include "mcl/occupancy_grid.hpp"
#include "mcl/sensor_model.hpp"
#include "mcl/types.hpp"

namespace mcl {

// Low-variance (systematic) resampling: Probabilistic Robotics, Table 4.4.
//
// A free function so it can be tested directly -- the index-advance bound is
// where an earlier version had an out-of-bounds read. `startOffset` must lie in
// [0, 1/n); the caller draws it. Input weights are assumed normalized.
// Output weights are reset to 1/n, because after resampling the cloud's
// density carries the distribution and the weights must not also carry it.
void lowVarianceResample(const std::vector<Particle>& source,
                         std::vector<Particle>& destination, double startOffset);

// Monte Carlo Localization over a known 2D occupancy grid.
// Thrun, Burgard & Fox, "Probabilistic Robotics", ch. 8.
//
// Contains no ROS types: the node translates messages at the boundary. That
// is what makes the models testable, and what would have made the ROS 2 port
// nearly free.
class ParticleFilter {
 public:
  struct Params {
    // 1000 is enough for reliable tracking (measured 8/8 seeds, mean final
    // error 0.055 m on the shipped map, and still 8/8 at 200). Global
    // localization from a uniform cloud needs about 5000 to be dependable --
    // 8/8 seeds there against 4/8 at 2000. See the README.
    int particleCount = 1000;

    // Only update when the robot has actually moved; otherwise repeated
    // measurement updates on identical data artificially sharpen the cloud.
    double minTranslationForUpdate = 0.02;  // m
    double minRotationForUpdate = 0.02;     // rad

    // Resample only when the effective sample size falls below this fraction
    // of the particle count. Resampling on every update (as an earlier version did)
    // throws away diversity and converges prematurely.
    double resampleThresholdRatio = 0.5;

    // Pose of the laser in the robot body frame.
    Pose2D laserOffset;

    std::uint32_t seed = 42;
  };

  ParticleFilter(const Params& params, const MotionModel::Params& motionParams,
                 const SensorModel::Params& sensorParams);

  // Replaces the map and rebuilds the likelihood field. Clears all derived
  // state, so calling it twice is safe -- an earlier version appended to the map,
  // the free/occupied lists and the particle vector without clearing, so a
  // second (latched) map message corrupted them and then wrote out of bounds.
  void setMap(std::shared_ptr<const OccupancyGrid> map);

  bool hasMap() const { return map_ != nullptr && !map_->empty(); }

  // Updates the body -> laser transform in place. Separate from the
  // constructor because the node only learns it once TF resolves, which can
  // happen after the cloud has already been initialized -- rebuilding the
  // filter at that point would throw the cloud away.
  void setLaserOffset(const Pose2D& offset) { params_.laserOffset = offset; }
  const Pose2D& laserOffset() const { return params_.laserOffset; }
  bool initialized() const { return initialized_; }

  // Uniformly over the free space of the map. Returns false if there is no
  // map or no free space to sample.
  bool initializeGlobal();

  // Gaussian cloud around a given pose, as /initialpose provides.
  bool initializeAtPose(const Pose2D& mean, double xyStdDev, double yawStdDev);

  // One filter cycle. Returns true if the motion threshold was met and an
  // update actually ran.
  bool update(const Pose2D& odom, const LaserScan& scan);

  // Weighted mean and covariance of the cloud, with a circular mean for yaw.
  PoseEstimate estimate() const;

  const std::vector<Particle>& particles() const { return particles_; }
  double effectiveSampleSize() const;

  // Number of times the filter had to reinitialize because every particle
  // became impossible. Useful as a health signal.
  int recoveryCount() const { return recoveryCount_; }

 private:
  // Converts accumulated log-likelihoods into normalized linear weights.
  // Returns false if the set is degenerate (no particle explains the scan).
  bool normalizeFromLogWeights(std::vector<double>& logWeights);
  void resample();

  Params params_;
  MotionModel motionModel_;
  SensorModel sensorModel_;

  std::shared_ptr<const OccupancyGrid> map_;
  LikelihoodField field_;

  std::vector<Particle> particles_;
  std::mt19937 rng_;

  bool initialized_ = false;
  // Member state behind an explicit flag. These were function-local statics,
  // zero-initialized, so the first update computed its delta from (0,0,0)
  // rather than from the first real odometry reading -- one enormous bogus
  // motion that scattered the cloud at startup.
  bool haveLastOdom_ = false;
  Pose2D lastOdom_;
  int recoveryCount_ = 0;
};

}  // namespace mcl

#endif  // MCL_PARTICLE_FILTER_HPP_
