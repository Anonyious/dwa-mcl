#include "mcl/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mcl {

OccupancyGrid::OccupancyGrid(int width, int height, double resolution,
                             const Pose2D& origin, std::vector<int8_t> data)
    : width_(width),
      height_(height),
      resolution_(resolution),
      origin_(origin),
      data_(std::move(data)) {
  const std::size_t expected =
      static_cast<std::size_t>(std::max(0, width_)) * static_cast<std::size_t>(std::max(0, height_));
  if (data_.size() != expected) {
    // Keep the object in a consistent empty state rather than allowing a
    // size mismatch to become an out-of-bounds read later.
    width_ = 0;
    height_ = 0;
    data_.clear();
  }
}

bool OccupancyGrid::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  if (resolution_ <= 0.0 || data_.empty()) {
    return false;
  }

  // Into the map frame, accounting for a rotated origin.
  const double dx = wx - origin_.x;
  const double dy = wy - origin_.y;
  const double c = std::cos(origin_.theta);
  const double s = std::sin(origin_.theta);
  const double mx = dx * c + dy * s;
  const double my = -dx * s + dy * c;

  // floor, not truncation: (int)(-0.3 / 1.0) is 0, which maps a point just
  // below the origin onto cell 0 instead of rejecting it.
  const double fx = std::floor(mx / resolution_);
  const double fy = std::floor(my / resolution_);

  // Check before narrowing, so a huge coordinate cannot overflow the cast.
  if (fx < 0.0 || fy < 0.0 || fx >= static_cast<double>(width_) ||
      fy >= static_cast<double>(height_)) {
    return false;
  }

  gx = static_cast<int>(fx);
  gy = static_cast<int>(fy);
  return true;
}

void OccupancyGrid::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  const double mx = (static_cast<double>(gx) + 0.5) * resolution_;
  const double my = (static_cast<double>(gy) + 0.5) * resolution_;
  const double c = std::cos(origin_.theta);
  const double s = std::sin(origin_.theta);
  wx = origin_.x + mx * c - my * s;
  wy = origin_.y + mx * s + my * c;
}

bool OccupancyGrid::isFreeAtWorld(double wx, double wy) const {
  int gx = 0;
  int gy = 0;
  if (!worldToGrid(wx, wy, gx, gy)) {
    return false;
  }
  return isFree(gx, gy);
}

void OccupancyGrid::worldBounds(double& minX, double& minY, double& maxX,
                                double& maxY) const {
  minX = minY = 0.0;
  maxX = maxY = 0.0;
  if (data_.empty()) {
    return;
  }

  // Transform all four corners, since the origin may be rotated.
  const double w = static_cast<double>(width_) * resolution_;
  const double h = static_cast<double>(height_) * resolution_;
  const double c = std::cos(origin_.theta);
  const double s = std::sin(origin_.theta);

  const double cornersX[4] = {0.0, w, w, 0.0};
  const double cornersY[4] = {0.0, 0.0, h, h};

  bool first = true;
  for (int i = 0; i < 4; ++i) {
    const double wx = origin_.x + cornersX[i] * c - cornersY[i] * s;
    const double wy = origin_.y + cornersX[i] * s + cornersY[i] * c;
    if (first) {
      minX = maxX = wx;
      minY = maxY = wy;
      first = false;
    } else {
      minX = std::min(minX, wx);
      maxX = std::max(maxX, wx);
      minY = std::min(minY, wy);
      maxY = std::max(maxY, wy);
    }
  }
}

}  // namespace mcl
