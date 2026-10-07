#ifndef MCL_TEST_HELPERS_HPP_
#define MCL_TEST_HELPERS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "mcl/likelihood_field.hpp"
#include "mcl/occupancy_grid.hpp"
#include "mcl/types.hpp"

namespace mcl {
namespace testing {

// A rectangular room: occupied border, free interior.
inline OccupancyGrid makeRoom(int width, int height, double resolution = 0.1,
                              const Pose2D& origin = Pose2D()) {
  std::vector<int8_t> cells(static_cast<std::size_t>(width) *
                                static_cast<std::size_t>(height),
                            OccupancyGrid::kFree);
  for (int x = 0; x < width; ++x) {
    cells[static_cast<std::size_t>(x)] = OccupancyGrid::kOccupied;
    cells[static_cast<std::size_t>(height - 1) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(x)] = OccupancyGrid::kOccupied;
  }
  for (int y = 0; y < height; ++y) {
    cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(width)] =
        OccupancyGrid::kOccupied;
    cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(width - 1)] = OccupancyGrid::kOccupied;
  }
  return OccupancyGrid(width, height, resolution, origin, std::move(cells));
}

// Brute-force squared distance to the nearest occupied cell, in m^2. The
// reference the O(n) transform is checked against.
inline std::vector<double> bruteForceSquaredDistances(const OccupancyGrid& map) {
  const double resSquared = map.resolution() * map.resolution();
  std::vector<double> out(map.cellCount(),
                          std::numeric_limits<double>::infinity());

  for (int y = 0; y < map.height(); ++y) {
    for (int x = 0; x < map.width(); ++x) {
      double best = std::numeric_limits<double>::infinity();
      for (int oy = 0; oy < map.height(); ++oy) {
        for (int ox = 0; ox < map.width(); ++ox) {
          if (!map.isOccupied(ox, oy)) {
            continue;
          }
          const double dx = static_cast<double>(x - ox);
          const double dy = static_cast<double>(y - oy);
          best = std::min(best, dx * dx + dy * dy);
        }
      }
      out[static_cast<std::size_t>(y) * static_cast<std::size_t>(map.width()) +
          static_cast<std::size_t>(x)] = best * resSquared;
    }
  }
  return out;
}

// Ray-cast one beam against the map, returning the hit distance or maxRange.
inline double raycast(const OccupancyGrid& map, double x, double y, double theta,
                      double maxRange) {
  const double step = map.resolution() * 0.25;
  for (double d = step; d < maxRange; d += step) {
    const double px = x + d * std::cos(theta);
    const double py = y + d * std::sin(theta);
    int gx = 0;
    int gy = 0;
    if (!map.worldToGrid(px, py, gx, gy)) {
      return maxRange;
    }
    if (map.isOccupied(gx, gy)) {
      return d;
    }
  }
  return maxRange;
}

// A noise-free scan from the given pose.
inline LaserScan simulateScan(const OccupancyGrid& map, const Pose2D& pose,
                              std::size_t beams = 180, double maxRange = 20.0) {
  LaserScan scan;
  scan.angleMin = -kPi;
  scan.angleIncrement = 2.0 * kPi / static_cast<double>(beams);
  scan.rangeMin = 0.01;
  scan.rangeMax = maxRange;
  scan.ranges.reserve(beams);
  for (std::size_t i = 0; i < beams; ++i) {
    const double angle = wrapToPi(pose.theta + scan.angleAt(i));
    scan.ranges.push_back(
        static_cast<float>(raycast(map, pose.x, pose.y, angle, maxRange)));
  }
  return scan;
}

}  // namespace testing
}  // namespace mcl

#endif  // MCL_TEST_HELPERS_HPP_
