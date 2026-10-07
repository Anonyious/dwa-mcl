#include "mcl/motion_model.hpp"

#include <cmath>

namespace mcl {

MotionModel::MotionModel(const Params& params, std::uint32_t seed)
    : params_(params), rng_(seed) {}

double MotionModel::sampleNormal(double standardDeviation) {
  if (standardDeviation <= 0.0) {
    return 0.0;
  }
  return standardDeviation * standardNormal_(rng_);
}

MotionModel::Deviations MotionModel::deviations(double rotation1,
                                                double translation,
                                                double rotation2) const {
  // Absolute values on every term.
  //
  // An earlier version omitted them, so a negative rotation produced a negative
  // standard deviation -- which then sign-flipped the sample drawn from it --
  // and in the translation term two opposite rotations cancelled instead of
  // accumulating noise.
  const double absRotation1 = std::abs(rotation1);
  const double absRotation2 = std::abs(rotation2);
  const double absTranslation = std::abs(translation);

  Deviations deviations;
  deviations.rotation1 = params_.alpha1 * absRotation1 + params_.alpha2 * absTranslation;
  deviations.translation =
      params_.alpha3 * absTranslation + params_.alpha4 * (absRotation1 + absRotation2);
  deviations.rotation2 = params_.alpha1 * absRotation2 + params_.alpha2 * absTranslation;
  return deviations;
}

Pose2D MotionModel::sample(const Pose2D& previousOdom, const Pose2D& currentOdom,
                           const Pose2D& particle) {
  const double dx = currentOdom.x - previousOdom.x;
  const double dy = currentOdom.y - previousOdom.y;
  const double translation = std::sqrt(dx * dx + dy * dy);
  const double deltaYaw = angleDiff(currentOdom.theta, previousOdom.theta);

  double rotation1 = 0.0;
  double rotation2 = deltaYaw;

  if (translation >= params_.minTranslation) {
    // Both rotations are wrapped. Unwrapped, a small physical rotation that
    // happens to straddle +-pi comes out near 2*pi, which multiplies into a
    // huge standard deviation and detonates the particle cloud.
    rotation1 = angleDiff(std::atan2(dy, dx), previousOdom.theta);
    rotation2 = angleDiff(deltaYaw, rotation1);
  }

  const Deviations sigma = deviations(rotation1, translation, rotation2);

  const double rotation1Hat = rotation1 - sampleNormal(sigma.rotation1);
  const double translationHat = translation - sampleNormal(sigma.translation);
  const double rotation2Hat = rotation2 - sampleNormal(sigma.rotation2);

  Pose2D result;
  result.x = particle.x + translationHat * std::cos(particle.theta + rotation1Hat);
  result.y = particle.y + translationHat * std::sin(particle.theta + rotation1Hat);
  // Wrapped, so yaw cannot grow without bound across a long run.
  result.theta = wrapToPi(particle.theta + rotation1Hat + rotation2Hat);
  return result;
}

}  // namespace mcl
