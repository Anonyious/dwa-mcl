#ifndef DWA_ANGLES_HPP_
#define DWA_ANGLES_HPP_

#include <cmath>

namespace dwa {

// M_PI is not guaranteed by <cmath> on MSVC without _USE_MATH_DEFINES, so we
// carry our own constant rather than depending on a platform extension.
inline constexpr double kPi = 3.14159265358979323846;

inline constexpr double degToRad(double degrees) {
  return degrees * kPi / 180.0;
}

// Wraps an angle into [-pi, pi]. Unlike a single if/else-if correction this is
// correct for arbitrarily large inputs (e.g. 5*pi).
inline double wrapToPi(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Signed smallest rotation taking `from` to `to`, in [-pi, pi].
inline double angleDiff(double to, double from) {
  return wrapToPi(to - from);
}

}  // namespace dwa

#endif  // DWA_ANGLES_HPP_
