#ifndef MCL_MOTION_MODEL_HPP_
#define MCL_MOTION_MODEL_HPP_

#include <cstdint>
#include <random>

#include "mcl/types.hpp"

namespace mcl {

// Odometry motion model: Thrun, Burgard & Fox, "Probabilistic Robotics",
// Table 5.6 (sample_motion_model_odometry).
//
// The odometry delta is decomposed into an initial rotation, a translation,
// and a final rotation; each is perturbed by zero-mean noise whose standard
// deviation is a linear function of the delta magnitudes.
class MotionModel {
 public:
  struct Params {
    // Noise coefficients. alpha1/alpha2 scale rotation noise, alpha3/alpha4
    // translation noise.
    double alpha1 = 0.05;
    double alpha2 = 0.05;
    double alpha3 = 0.05;
    double alpha4 = 0.05;

    // Below this translation the rotation decomposition is skipped: atan2 of
    // pure odometry noise yields an essentially random heading, which would
    // inject spurious rotation whenever the robot is nearly stationary.
    double minTranslation = 1e-3;  // m
  };

  MotionModel(const Params& params, std::uint32_t seed);

  // Propagates one particle through the odometry delta
  // (previousOdom -> currentOdom).
  Pose2D sample(const Pose2D& previousOdom, const Pose2D& currentOdom,
                const Pose2D& particle);

  // Standard deviations for a given delta, exposed for testing. All three are
  // guaranteed non-negative.
  struct Deviations {
    double rotation1 = 0.0;
    double translation = 0.0;
    double rotation2 = 0.0;
  };
  Deviations deviations(double rotation1, double translation,
                        double rotation2) const;

  const Params& params() const { return params_; }

 private:
  double sampleNormal(double standardDeviation);

  Params params_;
  // An earlier version used drand48(), which is POSIX-only and does not compile
  // under MSVC, and never seeded rand() -- so every run reused the same
  // "random" cloud by accident rather than on purpose.
  std::mt19937 rng_;
  std::normal_distribution<double> standardNormal_{0.0, 1.0};
};

}  // namespace mcl

#endif  // MCL_MOTION_MODEL_HPP_
