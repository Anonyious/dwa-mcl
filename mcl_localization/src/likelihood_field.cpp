#include "mcl/likelihood_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mcl {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

// Sentinel for "no feature here". Deliberately a large FINITE value rather
// than infinity: the lower-envelope step below subtracts two f values, and
// inf - inf is NaN, which then fails every comparison and silently corrupts
// the whole transform. Any row or column containing no occupied cell hits
// this, so it is the common case, not an edge case.
constexpr double kNoFeature = 1e20;

// Exact 1D squared distance transform (lower envelope of parabolas), in
// place. Felzenszwalb & Huttenlocher 2012, section 2.
//
// `f` holds the sampled function: 0 at feature points, +inf elsewhere. On
// return f[q] is min_p ( (q - p)^2 + f_in[p] ).
void distanceTransform1D(std::vector<double>& f, std::vector<int>& v,
                         std::vector<double>& z, std::vector<double>& out) {
  const int n = static_cast<int>(f.size());
  if (n == 0) {
    return;
  }

  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;

  for (int q = 1; q < n; ++q) {
    // Intersection of the parabolas rooted at q and at v[k].
    double s = ((f[q] + static_cast<double>(q) * q) -
                (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
               (2.0 * static_cast<double>(q) - 2.0 * static_cast<double>(v[k]));
    while (s <= z[k]) {
      // k cannot go below 0: z[0] is -inf, so s <= z[0] is always false.
      --k;
      s = ((f[q] + static_cast<double>(q) * q) -
           (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
          (2.0 * static_cast<double>(q) - 2.0 * static_cast<double>(v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<double>(q)) {
      ++k;
    }
    const double d = static_cast<double>(q) - static_cast<double>(v[k]);
    out[q] = d * d + f[v[k]];
  }
  std::copy(out.begin(), out.begin() + n, f.begin());
}

}  // namespace

LikelihoodField::LikelihoodField(const OccupancyGrid& map) {
  if (map.empty()) {
    return;
  }

  width_ = map.width();
  height_ = map.height();
  const std::size_t cells = static_cast<std::size_t>(width_) *
                            static_cast<std::size_t>(height_);
  squaredDistances_.assign(cells, kNoFeature);

  bool anyOccupied = false;
  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      if (map.isOccupied(gx, gy)) {
        squaredDistances_[static_cast<std::size_t>(gy) *
                              static_cast<std::size_t>(width_) +
                          static_cast<std::size_t>(gx)] = 0.0;
        anyOccupied = true;
      }
    }
  }

  if (!anyOccupied) {
    degenerate_ = true;
    return;
  }

  // Note: z[] below keeps real infinities as its outer boundaries. Those are
  // only ever compared, never subtracted, so they are safe.

  // Scratch buffers sized for the longer axis, reused across passes.
  const int longest = std::max(width_, height_);
  std::vector<double> line(static_cast<std::size_t>(longest));
  std::vector<int> v(static_cast<std::size_t>(longest));
  std::vector<double> z(static_cast<std::size_t>(longest) + 1);
  std::vector<double> out(static_cast<std::size_t>(longest));

  // Pass 1: along each row.
  for (int gy = 0; gy < height_; ++gy) {
    line.resize(static_cast<std::size_t>(width_));
    for (int gx = 0; gx < width_; ++gx) {
      line[static_cast<std::size_t>(gx)] = squaredDistances_
          [static_cast<std::size_t>(gy) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(gx)];
    }
    distanceTransform1D(line, v, z, out);
    for (int gx = 0; gx < width_; ++gx) {
      squaredDistances_[static_cast<std::size_t>(gy) *
                            static_cast<std::size_t>(width_) +
                        static_cast<std::size_t>(gx)] =
          line[static_cast<std::size_t>(gx)];
    }
  }

  // Pass 2: along each column. Composing the two gives the exact squared
  // Euclidean distance.
  for (int gx = 0; gx < width_; ++gx) {
    line.resize(static_cast<std::size_t>(height_));
    for (int gy = 0; gy < height_; ++gy) {
      line[static_cast<std::size_t>(gy)] = squaredDistances_
          [static_cast<std::size_t>(gy) * static_cast<std::size_t>(width_) +
           static_cast<std::size_t>(gx)];
    }
    distanceTransform1D(line, v, z, out);
    for (int gy = 0; gy < height_; ++gy) {
      squaredDistances_[static_cast<std::size_t>(gy) *
                            static_cast<std::size_t>(width_) +
                        static_cast<std::size_t>(gx)] =
          line[static_cast<std::size_t>(gy)];
    }
  }

  // Cells -> metres. Squared distance scales by resolution squared, and this
  // is the only place the conversion happens.
  const double resolutionSquared = map.resolution() * map.resolution();
  for (double& value : squaredDistances_) {
    value *= resolutionSquared;
  }
}

double LikelihoodField::squaredDistanceAtWorld(const OccupancyGrid& map,
                                               double wx, double wy,
                                               double outOfBoundsValue) const {
  if (squaredDistances_.empty()) {
    return outOfBoundsValue;
  }
  int gx = 0;
  int gy = 0;
  if (!map.worldToGrid(wx, wy, gx, gy)) {
    return outOfBoundsValue;
  }
  if (gx >= width_ || gy >= height_) {
    return outOfBoundsValue;
  }
  return squaredDistanceAt(gx, gy);
}

}  // namespace mcl
