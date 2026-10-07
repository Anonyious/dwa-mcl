#include "mcl/occupancy_grid.hpp"

#include <vector>

#include <gtest/gtest.h>

#include "test_helpers.hpp"

namespace {

mcl::OccupancyGrid makeFreeGrid(int w, int h, double res,
                                const mcl::Pose2D& origin = mcl::Pose2D()) {
  std::vector<int8_t> cells(static_cast<std::size_t>(w) * static_cast<std::size_t>(h),
                            mcl::OccupancyGrid::kFree);
  return mcl::OccupancyGrid(w, h, res, origin, std::move(cells));
}

TEST(OccupancyGrid, MapsCellCentresToTheirOwnCell) {
  const mcl::OccupancyGrid map = makeFreeGrid(10, 10, 0.5);
  for (int gy = 0; gy < 10; ++gy) {
    for (int gx = 0; gx < 10; ++gx) {
      double wx = 0.0;
      double wy = 0.0;
      map.gridToWorld(gx, gy, wx, wy);
      int rx = -1;
      int ry = -1;
      ASSERT_TRUE(map.worldToGrid(wx, wy, rx, ry)) << gx << "," << gy;
      EXPECT_EQ(rx, gx);
      EXPECT_EQ(ry, gy);
    }
  }
}

// Regression: every conversion in an earlier version was (int)(x / resolution),
// which ignores the map origin entirely. It only worked because the shipped
// map.yaml happens to have origin [0, 0, 0].
TEST(OccupancyGrid, HonoursANonZeroOrigin) {
  const mcl::OccupancyGrid map = makeFreeGrid(10, 10, 1.0, mcl::Pose2D(100.0, 50.0, 0.0));

  int gx = -1;
  int gy = -1;
  ASSERT_TRUE(map.worldToGrid(100.5, 50.5, gx, gy));
  EXPECT_EQ(gx, 0);
  EXPECT_EQ(gy, 0);

  ASSERT_TRUE(map.worldToGrid(109.5, 59.5, gx, gy));
  EXPECT_EQ(gx, 9);
  EXPECT_EQ(gy, 9);

  // A point at the world origin is far outside this map.
  EXPECT_FALSE(map.worldToGrid(0.0, 0.0, gx, gy));
}

// Regression: (int) truncates toward zero, so (int)(-0.3 / 1.0) == 0 maps a
// point just below the origin into cell 0 instead of rejecting it.
TEST(OccupancyGrid, FloorsRatherThanTruncatingNearTheOrigin) {
  const mcl::OccupancyGrid map = makeFreeGrid(10, 10, 1.0);
  int gx = -1;
  int gy = -1;

  EXPECT_TRUE(map.worldToGrid(0.5, 0.5, gx, gy));
  EXPECT_FALSE(map.worldToGrid(-0.3, 0.5, gx, gy)) << "truncation would accept this";
  EXPECT_FALSE(map.worldToGrid(0.5, -0.3, gx, gy));
  EXPECT_FALSE(map.worldToGrid(-0.001, -0.001, gx, gy));
}

TEST(OccupancyGrid, RejectsPointsBeyondTheFarEdge) {
  const mcl::OccupancyGrid map = makeFreeGrid(10, 10, 1.0);
  int gx = -1;
  int gy = -1;
  EXPECT_TRUE(map.worldToGrid(9.999, 9.999, gx, gy));
  EXPECT_FALSE(map.worldToGrid(10.0, 5.0, gx, gy));
  EXPECT_FALSE(map.worldToGrid(5.0, 10.0, gx, gy));
}

// A huge coordinate must be rejected before the narrowing cast, not after.
TEST(OccupancyGrid, RejectsCoordinatesThatWouldOverflowAnInt) {
  const mcl::OccupancyGrid map = makeFreeGrid(10, 10, 1.0);
  int gx = -1;
  int gy = -1;
  EXPECT_FALSE(map.worldToGrid(1e18, 1e18, gx, gy));
  EXPECT_FALSE(map.worldToGrid(-1e18, -1e18, gx, gy));
}

TEST(OccupancyGrid, HandlesARotatedOrigin) {
  // Origin rotated 90 degrees: the map's +x axis points along world +y.
  const mcl::OccupancyGrid map =
      makeFreeGrid(10, 4, 1.0, mcl::Pose2D(0.0, 0.0, 0.5 * mcl::kPi));

  int gx = -1;
  int gy = -1;
  // Map cell (5, 1) centre is at map (5.5, 1.5) -> world (-1.5, 5.5).
  ASSERT_TRUE(map.worldToGrid(-1.5, 5.5, gx, gy));
  EXPECT_EQ(gx, 5);
  EXPECT_EQ(gy, 1);

  double wx = 0.0;
  double wy = 0.0;
  map.gridToWorld(5, 1, wx, wy);
  EXPECT_NEAR(wx, -1.5, 1e-9);
  EXPECT_NEAR(wy, 5.5, 1e-9);
}

TEST(OccupancyGrid, IsFreeAtWorldIsFalseOffTheMap) {
  const mcl::OccupancyGrid map = makeFreeGrid(5, 5, 1.0);
  EXPECT_TRUE(map.isFreeAtWorld(2.5, 2.5));
  // Regression: an earlier version indexed the map with a particle pose before any
  // bounds check, so a particle pushed off the map was an OOB read.
  EXPECT_FALSE(map.isFreeAtWorld(-5.0, 2.5));
  EXPECT_FALSE(map.isFreeAtWorld(2.5, 100.0));
}

TEST(OccupancyGrid, UnknownCellsAreNotFree) {
  std::vector<int8_t> cells(4, mcl::OccupancyGrid::kUnknown);
  cells[0] = mcl::OccupancyGrid::kFree;
  const mcl::OccupancyGrid map(2, 2, 1.0, mcl::Pose2D(), std::move(cells));

  EXPECT_TRUE(map.isFreeAtWorld(0.5, 0.5));
  EXPECT_FALSE(map.isFreeAtWorld(1.5, 0.5));
  EXPECT_FALSE(map.isFreeAtWorld(0.5, 1.5));
}

TEST(OccupancyGrid, RejectsAMismatchedDataSize) {
  // 3x3 declared, 4 cells supplied: must end up empty rather than letting the
  // mismatch become an out-of-bounds read later.
  const mcl::OccupancyGrid map(3, 3, 1.0, mcl::Pose2D(), std::vector<int8_t>(4, 0));
  EXPECT_TRUE(map.empty());
  int gx = 0;
  int gy = 0;
  EXPECT_FALSE(map.worldToGrid(0.5, 0.5, gx, gy));
}

TEST(OccupancyGrid, WorldBoundsCoverTheWholeMap) {
  const mcl::OccupancyGrid map = makeFreeGrid(10, 20, 0.5, mcl::Pose2D(-3.0, 7.0, 0.0));
  double minX = 0.0;
  double minY = 0.0;
  double maxX = 0.0;
  double maxY = 0.0;
  map.worldBounds(minX, minY, maxX, maxY);

  EXPECT_NEAR(minX, -3.0, 1e-9);
  EXPECT_NEAR(minY, 7.0, 1e-9);
  EXPECT_NEAR(maxX, -3.0 + 5.0, 1e-9);   // 10 cells * 0.5
  EXPECT_NEAR(maxY, 7.0 + 10.0, 1e-9);   // 20 cells * 0.5
}

TEST(OccupancyGrid, RoomHelperHasAnOccupiedBorderAndFreeInterior) {
  const mcl::OccupancyGrid room = mcl::testing::makeRoom(8, 6, 1.0);
  EXPECT_TRUE(room.isOccupied(0, 0));
  EXPECT_TRUE(room.isOccupied(7, 5));
  EXPECT_TRUE(room.isOccupied(3, 0));
  EXPECT_TRUE(room.isFree(3, 3));
  EXPECT_TRUE(room.isFree(1, 1));
}

}  // namespace
