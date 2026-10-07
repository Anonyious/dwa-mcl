#ifndef MCL_ANGLES_HPP_
#define MCL_ANGLES_HPP_

#include <cmath>
#include <cstddef>

namespace mcl {

inline constexpr double kPi = 3.14159265358979323846;

// Wraps an angle into [-pi, pi]. Correct for arbitrarily large inputs; the
// original fromPiToMinusPi used a single if/else-if and so only ever removed
// one revolution (5*pi came back as 3*pi, still out of range).
inline double wrapToPi(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Signed smallest rotation taking `from` to `to`, in [-pi, pi].
inline double angleDiff(double to, double from) {
  return wrapToPi(to - from);
}

// Circular (vector) mean of weighted angles: atan2(sum w*sin, sum w*cos).
//
// Averaging angles arithmetically is wrong near the +-pi branch cut -- the
// mean of +3.1 and -3.1 is 0, pointing the opposite way from both inputs.
// Returns 0 when the resultant vector has no direction (perfectly opposed
// angles, or zero total weight).
template <typename Iterator, typename AngleFn, typename WeightFn>
double circularMean(Iterator begin, Iterator end, AngleFn angleOf,
                    WeightFn weightOf) {
  double sumSin = 0.0;
  double sumCos = 0.0;
  for (Iterator it = begin; it != end; ++it) {
    const double weight = weightOf(*it);
    const double angle = angleOf(*it);
    sumSin += weight * std::sin(angle);
    sumCos += weight * std::cos(angle);
  }
  if (std::abs(sumSin) < 1e-12 && std::abs(sumCos) < 1e-12) {
    return 0.0;
  }
  return std::atan2(sumSin, sumCos);
}

}  // namespace mcl

#endif  // MCL_ANGLES_HPP_
