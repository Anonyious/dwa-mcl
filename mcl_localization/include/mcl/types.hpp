#ifndef MCL_TYPES_HPP_
#define MCL_TYPES_HPP_

#include <array>
#include <vector>

#include "mcl/angles.hpp"

namespace mcl {

// Planar pose. Deliberately a plain struct rather than a ROS message or an
// Eigen vector, so the algorithm layer has no dependency on either.
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;

  Pose2D() = default;
  Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

// Composes a body-frame offset onto a pose: the full planar rigid transform.
//
// Applying a sensor offset as
//   sx = x + ox*cos(theta);  sy = y + oy*sin(theta);
// drops the oy term from x and the ox term from y. For an offset like
// (0.75, -0.23) that misplaces the laser by up to ~0.8 m, which is far
// larger than a typical map resolution.
inline Pose2D compose(const Pose2D& base, const Pose2D& offset) {
  const double c = std::cos(base.theta);
  const double s = std::sin(base.theta);
  return Pose2D(base.x + offset.x * c - offset.y * s,
                base.y + offset.x * s + offset.y * c,
                wrapToPi(base.theta + offset.theta));
}

struct Particle {
  Pose2D pose;
  double weight = 0.0;  // explicitly initialized; an earlier version left it garbage
};

// A laser scan, stripped of ROS types. Angles and ranges come from the
// message rather than being hardcoded.
struct LaserScan {
  double angleMin = 0.0;
  double angleIncrement = 0.0;
  double rangeMin = 0.0;
  double rangeMax = 0.0;
  std::vector<float> ranges;

  double angleAt(std::size_t index) const {
    return angleMin + angleIncrement * static_cast<double>(index);
  }
};

// Weighted mean pose plus its covariance, in the (x, y, theta) ordering ROS
// uses. Publishing the covariance makes a multi-modal particle cloud visible
// instead of silently collapsing it to a mean no particle occupies.
struct PoseEstimate {
  Pose2D mean;
  std::array<double, 9> covariance{};  // row-major 3x3
  double effectiveSampleSize = 0.0;
  bool valid = false;
};

}  // namespace mcl

#endif  // MCL_TYPES_HPP_
