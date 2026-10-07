#include "mcl/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mcl {

ParticleFilter::ParticleFilter(const Params& params,
                               const MotionModel::Params& motionParams,
                               const SensorModel::Params& sensorParams)
    : params_(params),
      motionModel_(motionParams, params.seed),
      sensorModel_(sensorParams),
      rng_(params.seed ^ 0x9e3779b9u) {
  if (params_.particleCount < 1) {
    params_.particleCount = 1;
  }
}

void ParticleFilter::setMap(std::shared_ptr<const OccupancyGrid> map) {
  map_ = std::move(map);
  field_ = map_ ? LikelihoodField(*map_) : LikelihoodField();

  // Everything derived from the old map is now stale.
  particles_.clear();
  initialized_ = false;
  haveLastOdom_ = false;
}

bool ParticleFilter::initializeGlobal() {
  if (!hasMap()) {
    return false;
  }

  double minX = 0.0;
  double minY = 0.0;
  double maxX = 0.0;
  double maxY = 0.0;
  map_->worldBounds(minX, minY, maxX, maxY);

  std::uniform_real_distribution<double> xDist(minX, maxX);
  std::uniform_real_distribution<double> yDist(minY, maxY);
  std::uniform_real_distribution<double> yawDist(-kPi, kPi);

  particles_.clear();
  particles_.reserve(static_cast<std::size_t>(params_.particleCount));

  // Bounded rejection sampling: a map that is almost entirely occupied would
  // otherwise spin here forever.
  const int maxAttempts = params_.particleCount * 1000;
  int attempts = 0;
  const double uniformWeight = 1.0 / static_cast<double>(params_.particleCount);

  while (static_cast<int>(particles_.size()) < params_.particleCount &&
         attempts < maxAttempts) {
    ++attempts;
    const double x = xDist(rng_);
    const double y = yDist(rng_);
    if (!map_->isFreeAtWorld(x, y)) {
      continue;
    }
    Particle particle;
    particle.pose = Pose2D(x, y, yawDist(rng_));
    particle.weight = uniformWeight;
    particles_.push_back(particle);
  }

  if (particles_.empty()) {
    return false;
  }

  // If rejection sampling fell short, even out the weights over what we got.
  const double weight = 1.0 / static_cast<double>(particles_.size());
  for (Particle& particle : particles_) {
    particle.weight = weight;
  }

  initialized_ = true;
  haveLastOdom_ = false;
  return true;
}

bool ParticleFilter::initializeAtPose(const Pose2D& mean, double xyStdDev,
                                     double yawStdDev) {
  if (!hasMap()) {
    return false;
  }

  std::normal_distribution<double> xyNoise(0.0, std::max(1e-6, xyStdDev));
  std::normal_distribution<double> yawNoise(0.0, std::max(1e-6, yawStdDev));

  particles_.clear();
  particles_.reserve(static_cast<std::size_t>(params_.particleCount));

  const double uniformWeight = 1.0 / static_cast<double>(params_.particleCount);
  for (int i = 0; i < params_.particleCount; ++i) {
    Particle particle;
    particle.pose = Pose2D(mean.x + xyNoise(rng_), mean.y + xyNoise(rng_),
                           wrapToPi(mean.theta + yawNoise(rng_)));
    particle.weight = uniformWeight;
    particles_.push_back(particle);
  }

  initialized_ = true;
  haveLastOdom_ = false;
  return true;
}

double ParticleFilter::effectiveSampleSize() const {
  double sumSquared = 0.0;
  for (const Particle& particle : particles_) {
    sumSquared += particle.weight * particle.weight;
  }
  if (sumSquared <= 0.0) {
    return 0.0;
  }
  return 1.0 / sumSquared;
}

bool ParticleFilter::normalizeFromLogWeights(std::vector<double>& logWeights) {
  if (logWeights.empty()) {
    return false;
  }

  // Subtract the maximum before exponentiating, so a very negative set of
  // log-likelihoods does not underflow to all zeros.
  const double maxLog = *std::max_element(logWeights.begin(), logWeights.end());
  if (!std::isfinite(maxLog)) {
    return false;
  }

  double total = 0.0;
  for (std::size_t i = 0; i < logWeights.size(); ++i) {
    const double weight = std::exp(logWeights[i] - maxLog);
    particles_[i].weight = weight;
    total += weight;
  }

  // Guard the division. An earlier version divided unconditionally, so an all-zero
  // weight set produced NaN weights, after which the resampler collapsed the
  // entire cloud onto particle 0.
  if (!(total > 0.0) || !std::isfinite(total)) {
    return false;
  }

  for (Particle& particle : particles_) {
    particle.weight /= total;
  }
  return true;
}

bool ParticleFilter::update(const Pose2D& odom, const LaserScan& scan) {
  if (!initialized_ || !hasMap() || particles_.empty()) {
    return false;
  }

  if (!haveLastOdom_) {
    // Establish the reference from the first real reading, not from (0,0,0).
    lastOdom_ = odom;
    haveLastOdom_ = true;
    return false;
  }

  const double dx = odom.x - lastOdom_.x;
  const double dy = odom.y - lastOdom_.y;
  const double translation = std::sqrt(dx * dx + dy * dy);
  const double rotation = std::abs(angleDiff(odom.theta, lastOdom_.theta));

  if (translation < params_.minTranslationForUpdate &&
      rotation < params_.minRotationForUpdate) {
    return false;
  }

  // --- Motion update: propagate in place, no temporary vector ---
  for (Particle& particle : particles_) {
    particle.pose = motionModel_.sample(lastOdom_, odom, particle.pose);
  }
  lastOdom_ = odom;

  // --- Measurement update ---
  if (!scan.ranges.empty()) {
    std::vector<double> logWeights(particles_.size(), 0.0);
    for (std::size_t i = 0; i < particles_.size(); ++i) {
      const Pose2D laserPose = compose(particles_[i].pose, params_.laserOffset);
      logWeights[i] =
          sensorModel_.logLikelihood(laserPose, scan, *map_, field_);
    }

    if (!normalizeFromLogWeights(logWeights)) {
      // Nothing explains the scan: fall back to global localization rather
      // than propagating NaN weights through the resampler.
      ++recoveryCount_;
      initializeGlobal();
      return true;
    }

    // --- Adaptive resampling ---
    const double threshold =
        params_.resampleThresholdRatio * static_cast<double>(particles_.size());
    if (effectiveSampleSize() < threshold) {
      resample();
    }
  }

  return true;
}

void lowVarianceResample(const std::vector<Particle>& source,
                         std::vector<Particle>& destination, double startOffset) {
  const std::size_t n = source.size();
  destination.resize(n);
  if (n == 0) {
    return;
  }

  const double step = 1.0 / static_cast<double>(n);
  double cumulative = source[0].weight;
  std::size_t i = 0;

  for (std::size_t m = 0; m < n; ++m) {
    const double target = startOffset + static_cast<double>(m) * step;
    // The guard is on i, the index being advanced.
    //
    // An earlier version guarded on the *outer* loop variable, which never changes
    // inside the inner loop -- so the condition was constant and i could walk
    // past the last particle. Since the weights sum to 1.0 only up to
    // floating-point error, `target > cumulative` at the final particle is a
    // live possibility, not a theoretical one.
    while (target > cumulative && i + 1 < n) {
      ++i;
      cumulative += source[i].weight;
    }
    destination[m] = source[i];
    destination[m].weight = step;
  }
}

void ParticleFilter::resample() {
  if (particles_.empty()) {
    return;
  }
  const double step = 1.0 / static_cast<double>(particles_.size());
  std::uniform_real_distribution<double> startDist(0.0, step);
  const double start = startDist(rng_);

  const std::vector<Particle> source = particles_;
  lowVarianceResample(source, particles_, start);
}

PoseEstimate ParticleFilter::estimate() const {
  PoseEstimate estimate;
  if (particles_.empty()) {
    return estimate;
  }

  double totalWeight = 0.0;
  for (const Particle& particle : particles_) {
    totalWeight += particle.weight;
  }
  if (!(totalWeight > 0.0)) {
    return estimate;
  }

  // Weighted mean position. Accumulators explicitly zeroed -- an earlier version
  // declared `double x, y, yaw;` uninitialized and then accumulated into
  // them, so the published pose and TF were undefined behaviour.
  double meanX = 0.0;
  double meanY = 0.0;
  for (const Particle& particle : particles_) {
    meanX += particle.weight * particle.pose.x;
    meanY += particle.weight * particle.pose.y;
  }
  meanX /= totalWeight;
  meanY /= totalWeight;

  // Circular mean for yaw, and weight-aware -- an earlier version took a plain
  // arithmetic mean over particles and ignored the weights entirely.
  const double meanYaw = circularMean(
      particles_.begin(), particles_.end(),
      [](const Particle& p) { return p.pose.theta; },
      [](const Particle& p) { return p.weight; });

  estimate.mean = Pose2D(meanX, meanY, meanYaw);

  double varXX = 0.0;
  double varXY = 0.0;
  double varYY = 0.0;
  double varYaw = 0.0;
  for (const Particle& particle : particles_) {
    const double ex = particle.pose.x - meanX;
    const double ey = particle.pose.y - meanY;
    const double eYaw = angleDiff(particle.pose.theta, meanYaw);
    varXX += particle.weight * ex * ex;
    varXY += particle.weight * ex * ey;
    varYY += particle.weight * ey * ey;
    varYaw += particle.weight * eYaw * eYaw;
  }
  varXX /= totalWeight;
  varXY /= totalWeight;
  varYY /= totalWeight;
  varYaw /= totalWeight;

  estimate.covariance = {varXX, varXY, 0.0, varXY, varYY, 0.0, 0.0, 0.0, varYaw};
  estimate.effectiveSampleSize = effectiveSampleSize();
  estimate.valid = true;
  return estimate;
}

}  // namespace mcl
