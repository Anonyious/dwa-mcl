#ifndef MCL_OCCUPANCY_GRID_HPP_
#define MCL_OCCUPANCY_GRID_HPP_

#include <cstdint>
#include <vector>

#include "mcl/types.hpp"

namespace mcl {

// A 2D occupancy grid with origin-aware, bounds-checked coordinate
// conversion.
//
// The naive conversion is a bare `(int)(x / resolution)`: no map origin, no
// floor (truncation toward zero is wrong for negative coordinates), and no
// bounds check -- which matters because the motion model can push a particle
// off the map. Routing every conversion through this class fixes all three at
// the source.
class OccupancyGrid {
 public:
  // ROS occupancy values.
  static constexpr int8_t kFree = 0;
  static constexpr int8_t kOccupied = 100;
  static constexpr int8_t kUnknown = -1;

  OccupancyGrid() = default;

  OccupancyGrid(int width, int height, double resolution, const Pose2D& origin,
                std::vector<int8_t> data);

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  const Pose2D& origin() const { return origin_; }
  bool empty() const { return data_.empty(); }
  std::size_t cellCount() const { return data_.size(); }

  bool inBounds(int gx, int gy) const {
    return gx >= 0 && gy >= 0 && gx < width_ && gy < height_;
  }

  // World -> grid cell. Returns false (and leaves gx/gy untouched) when the
  // point falls outside the map. Handles a rotated map origin and floors
  // rather than truncating.
  bool worldToGrid(double wx, double wy, int& gx, int& gy) const;

  // Centre of the given cell, in world coordinates.
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  // Precondition: inBounds(gx, gy).
  int8_t at(int gx, int gy) const {
    return data_[static_cast<std::size_t>(gy) * static_cast<std::size_t>(width_) +
                 static_cast<std::size_t>(gx)];
  }

  bool isOccupied(int gx, int gy) const { return at(gx, gy) == kOccupied; }
  bool isFree(int gx, int gy) const { return at(gx, gy) == kFree; }

  // Out-of-bounds and unknown both count as not free.
  bool isFreeAtWorld(double wx, double wy) const;

  const std::vector<int8_t>& data() const { return data_; }

  // World-frame axis-aligned extent of the map, for uniform sampling.
  void worldBounds(double& minX, double& minY, double& maxX, double& maxY) const;

 private:
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.0;
  Pose2D origin_;
  // Flat storage. An earlier version used vector<vector<int>>, which for a
  // realistic 2000x2000 map is 4 M ints plus 2000 per-row allocations.
  std::vector<int8_t> data_;
};

}  // namespace mcl

#endif  // MCL_OCCUPANCY_GRID_HPP_
