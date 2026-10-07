#include "mcl/likelihood_field.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "test_helpers.hpp"

namespace {

TEST(LikelihoodField, MatchesBruteForceExactly) {
  const mcl::OccupancyGrid room = mcl::testing::makeRoom(17, 13, 0.1);
  const mcl::LikelihoodField field(room);
  const std::vector<double> reference = mcl::testing::bruteForceSquaredDistances(room);

  ASSERT_FALSE(field.empty());
  for (int gy = 0; gy < room.height(); ++gy) {
    for (int gx = 0; gx < room.width(); ++gx) {
      const double expected =
          reference[static_cast<std::size_t>(gy) *
                        static_cast<std::size_t>(room.width()) +
                    static_cast<std::size_t>(gx)];
      EXPECT_NEAR(field.squaredDistanceAt(gx, gy), expected, 1e-9)
          << "cell " << gx << "," << gy;
    }
  }
}

// Regression: the two-pass transform subtracts two sampled values. Seeding
// non-feature cells with real infinity makes that inf - inf == NaN for any
// row containing no occupied cell, and NaN then fails every comparison and
// silently corrupts the entire field. A single obstacle in the corner leaves
// most rows empty, so this is the common case.
TEST(LikelihoodField, IsFiniteAndCorrectWithASingleObstacle) {
  std::vector<int8_t> cells(20 * 20, mcl::OccupancyGrid::kFree);
  cells[0] = mcl::OccupancyGrid::kOccupied;  // cell (0, 0) only
  const mcl::OccupancyGrid map(20, 20, 1.0, mcl::Pose2D(), std::move(cells));

  const mcl::LikelihoodField field(map);
  ASSERT_FALSE(field.empty());
  ASSERT_FALSE(field.degenerate());

  for (int gy = 0; gy < 20; ++gy) {
    for (int gx = 0; gx < 20; ++gx) {
      const double value = field.squaredDistanceAt(gx, gy);
      ASSERT_FALSE(std::isnan(value)) << "NaN at " << gx << "," << gy;
      const double expected = static_cast<double>(gx * gx + gy * gy);
      EXPECT_NEAR(value, expected, 1e-9) << "cell " << gx << "," << gy;
    }
  }
}

// Regression: an earlier version stored squared distance in CELLS and never scaled
// by the resolution, then squared it again inside the Gaussian.
TEST(LikelihoodField, IsInSquareMetresNotCells) {
  std::vector<int8_t> cells(5 * 5, mcl::OccupancyGrid::kFree);
  cells[0] = mcl::OccupancyGrid::kOccupied;
  const mcl::OccupancyGrid map(5, 5, 0.5, mcl::Pose2D(), std::move(cells));
  const mcl::LikelihoodField field(map);

  // One cell away at 0.5 m/cell is 0.5 m, so 0.25 m^2 -- not 1.0.
  EXPECT_NEAR(field.squaredDistanceAt(1, 0), 0.25, 1e-12);
  EXPECT_NEAR(std::sqrt(field.squaredDistanceAt(1, 0)), 0.5, 1e-12);

  // The diagonal neighbour is sqrt(2) cells = 0.707 m.
  EXPECT_NEAR(field.squaredDistanceAt(1, 1), 0.5, 1e-12);
  EXPECT_NEAR(std::sqrt(field.squaredDistanceAt(1, 1)), std::sqrt(0.5), 1e-12);

  EXPECT_NEAR(field.squaredDistanceAt(0, 0), 0.0, 1e-12);
}

TEST(LikelihoodField, OccupiedCellsAreAtZeroDistance) {
  const mcl::OccupancyGrid room = mcl::testing::makeRoom(10, 10, 0.25);
  const mcl::LikelihoodField field(room);
  for (int gy = 0; gy < room.height(); ++gy) {
    for (int gx = 0; gx < room.width(); ++gx) {
      if (room.isOccupied(gx, gy)) {
        EXPECT_NEAR(field.squaredDistanceAt(gx, gy), 0.0, 1e-12);
      }
    }
  }
}

TEST(LikelihoodField, LayoutMatchesTheMapOnANonSquareMap) {
  // Regression: an earlier version allocated [height][width] but indexed [x][y],
  // which is self-consistent only on a square map and an out-of-bounds read
  // on any other.
  std::vector<int8_t> cells(40 * 7, mcl::OccupancyGrid::kFree);
  cells[0] = mcl::OccupancyGrid::kOccupied;             // (0, 0)
  const mcl::OccupancyGrid map(40, 7, 1.0, mcl::Pose2D(), std::move(cells));
  const mcl::LikelihoodField field(map);

  EXPECT_EQ(field.width(), 40);
  EXPECT_EQ(field.height(), 7);
  // 30 cells along x must read as 900, not as an out-of-range row.
  EXPECT_NEAR(field.squaredDistanceAt(30, 0), 900.0, 1e-9);
  EXPECT_NEAR(field.squaredDistanceAt(0, 6), 36.0, 1e-9);
}

TEST(LikelihoodField, ReportsDegenerateWhenNothingIsOccupied) {
  std::vector<int8_t> cells(6 * 6, mcl::OccupancyGrid::kFree);
  const mcl::OccupancyGrid map(6, 6, 1.0, mcl::Pose2D(), std::move(cells));
  const mcl::LikelihoodField field(map);
  EXPECT_TRUE(field.degenerate());
}

TEST(LikelihoodField, EmptyMapYieldsEmptyField) {
  const mcl::LikelihoodField field(mcl::OccupancyGrid{});
  EXPECT_TRUE(field.empty());
}

TEST(LikelihoodField, WorldLookupReturnsTheSentinelOffTheMap) {
  const mcl::OccupancyGrid room = mcl::testing::makeRoom(10, 10, 0.5);
  const mcl::LikelihoodField field(room);
  const double sentinel = -12345.0;

  EXPECT_NE(field.squaredDistanceAtWorld(room, 2.5, 2.5, sentinel), sentinel);
  EXPECT_EQ(field.squaredDistanceAtWorld(room, -50.0, 2.5, sentinel), sentinel);
  EXPECT_EQ(field.squaredDistanceAtWorld(room, 2.5, 500.0, sentinel), sentinel);
}

TEST(LikelihoodField, WorldLookupHonoursTheMapOrigin) {
  const mcl::OccupancyGrid room =
      mcl::testing::makeRoom(10, 10, 1.0, mcl::Pose2D(1000.0, -500.0, 0.0));
  const mcl::LikelihoodField field(room);
  const double sentinel = -1.0;

  // Interior point in world coordinates.
  const double value = field.squaredDistanceAtWorld(room, 1005.5, -495.5, sentinel);
  EXPECT_NE(value, sentinel);
  EXPECT_GE(value, 0.0);

  // The world origin is nowhere near this map.
  EXPECT_EQ(field.squaredDistanceAtWorld(room, 0.0, 0.0, sentinel), sentinel);
}

}  // namespace
