#ifndef MCL_SENSOR_MODEL_HPP_
#define MCL_SENSOR_MODEL_HPP_

#include <cstddef>

#include "mcl/likelihood_field.hpp"
#include "mcl/occupancy_grid.hpp"
#include "mcl/types.hpp"

namespace mcl {

// Likelihood-field range-finder model: Thrun, Burgard & Fox,
// "Probabilistic Robotics", Table 6.3 (likelihood_field_range_finder_model).
//
// Each beam endpoint is projected into the map and scored by its distance to
// the nearest obstacle, mixed with a uniform term:
//
//   p(z_k) = zHit * N(dist; 0, sigmaHit) + zRand / rangeMax
//
// Per-beam likelihoods are combined in log space and, by default, divided by
// the beam count -- the geometric mean per-beam likelihood rather than the
// raw product. logLikelihood() explains why.
//
// An earlier version summed the per-beam likelihoods starting from an arbitrary
// weight of 1, which is neither the textbook product nor AMCL's deliberate
// sum of cubes, and yields a weight that is roughly "how many beams landed
// near a wall" -- a much weaker statistic that barely discriminates between
// poses.
class SensorModel {
 public:
  struct Params {
    double zHit = 0.9;
    double zRand = 0.1;
    double sigmaHit = 0.1;  // m; measured optimum (see docs/algorithm.md)

    // Use every Nth beam. A few thousand particles times a full 720-beam
    // scan is millions of exp() calls per update, which will not run at
    // 10 Hz; AMCL uses a few dozen beams.
    int beamSkip = 12;

    // Ignore returns beyond this range (0 = use the scan's own range_max).
    double maxRange = 0.0;

    // Divide the summed log-likelihood by the number of beams scored, giving
    // the geometric mean per-beam likelihood instead of the raw product.
    // See the long comment in logLikelihood() -- the raw product is too
    // peaked for global localization and makes beamSkip retune the filter's
    // confidence as a side effect. Set false for the textbook product.
    bool normalizeByBeamCount = true;

    // A particle whose own pose or laser origin is not in free space gets
    // this log-likelihood rather than a hard zero, so the filter can still
    // rank hopeless particles against each other instead of producing an
    // all-zero weight set.
    double invalidPoseLogLikelihood = -1e3;
  };

  explicit SensorModel(const Params& params) : params_(params) {}

  // Log-likelihood of `scan` given that the LASER (not the robot body) is at
  // `laserPose`. Always finite.
  double logLikelihood(const Pose2D& laserPose, const LaserScan& scan,
                       const OccupancyGrid& map,
                       const LikelihoodField& field) const;

  // Number of beams this model would actually use for the given scan.
  std::size_t beamsUsed(const LaserScan& scan) const;

  const Params& params() const { return params_; }

 private:
  Params params_;
};

}  // namespace mcl

#endif  // MCL_SENSOR_MODEL_HPP_
