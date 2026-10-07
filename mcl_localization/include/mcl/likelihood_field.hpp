#ifndef MCL_LIKELIHOOD_FIELD_HPP_
#define MCL_LIKELIHOOD_FIELD_HPP_

#include <vector>

#include "mcl/occupancy_grid.hpp"

namespace mcl {

// Squared distance from every cell to the nearest occupied cell, in METRES
// SQUARED.
//
// Two bugs are worth recording here, both found the hard way.
//
// 1. Units. Storing (dy_cells^2 + dx_cells^2) -- squared distance in
//    *cells* -- and feeding it straight into a Gaussian evaluated as
//    exp(-x^2 / 2*sigma^2) computes exp(-d^4 / 2*sigma^2) with d in cells,
//    not exp(-d^2 / 2*sigma^2) with d in metres. The field then decays to
//    nothing within about a cell, and the filter needs thousands of
//    particles to localize at all. This class stores the true squared
//    distance in metres and the sensor model squares sigma, so the squaring
//    happens exactly once.
//
// 2. Cost. Comparing every free cell against every occupied cell is
//    O(free x occupied): tolerable on a small map, but ~10^10 operations on
//    a realistic 2000x2000 one, i.e. minutes to hours of startup.
//    This uses the exact O(n) two-pass squared Euclidean distance transform of
//    Felzenszwalb & Huttenlocher, "Distance Transforms of Sampled Functions",
//    Theory of Computing 8(19), 2012.
class LikelihoodField {
 public:
  LikelihoodField() = default;

  // Builds the field for `map`. Unknown cells are treated as not occupied.
  explicit LikelihoodField(const OccupancyGrid& map);

  bool empty() const { return squaredDistances_.empty(); }
  int width() const { return width_; }
  int height() const { return height_; }

  // Squared distance in m^2 from the given cell to the nearest occupied cell.
  // Precondition: the cell is in bounds.
  double squaredDistanceAt(int gx, int gy) const {
    return squaredDistances_[static_cast<std::size_t>(gy) *
                                 static_cast<std::size_t>(width_) +
                             static_cast<std::size_t>(gx)];
  }

  // World-coordinate lookup. Returns `outOfBoundsValue` for points off the
  // map, so a beam landing outside cannot read out of bounds.
  double squaredDistanceAtWorld(const OccupancyGrid& map, double wx, double wy,
                                double outOfBoundsValue) const;

  // True when the map contained no occupied cells at all, in which case every
  // distance is the sentinel and the field carries no information.
  bool degenerate() const { return degenerate_; }

 private:
  int width_ = 0;
  int height_ = 0;
  bool degenerate_ = false;
  // Row-major, same layout as OccupancyGrid. Allocating [yMax][xMax] but
  // indexing [x][y] is self-consistent on a square map and an out-of-bounds
  // read on any other, so the layouts are kept deliberately identical.
  std::vector<double> squaredDistances_;
};

}  // namespace mcl

#endif  // MCL_LIKELIHOOD_FIELD_HPP_
