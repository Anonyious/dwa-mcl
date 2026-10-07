#ifndef DWA_VISUALIZER_HPP_
#define DWA_VISUALIZER_HPP_

#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "dwa/types.hpp"

namespace dwa {

// OpenCV rendering for the planner. Kept separate from DwaPlanner so the
// algorithm has no graphics dependency and can be unit-tested.
class Visualizer {
 public:
  struct Options {
    std::string windowName = "Dynamic Window Approach: Motion Planner";
    int canvasSize = 800;       // px, square
    double marginMeters = 3.0;  // world-space padding around the fitted extent
  };

  // The view is fitted once, at construction, to the bounding box of the
  // obstacles plus the start and goal. The pixels-per-metre scale is derived
  // from that extent rather than hardcoded.
  Visualizer(Options options, const std::vector<Obstacle>& obstacles,
             const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
             double robotRadius, double collisionRadius);

  ~Visualizer();

  void render(const State& state, const Trajectory& trajectory,
              const std::vector<Obstacle>& obstacles,
              const Eigen::Vector2d& goal, bool recovering);

  // Returns the key code, or -1 if none. 27 is ESC.
  int show(int waitMs);

  double pixelsPerMeter() const { return scale_; }

 private:
  cv::Point toPixel(double x, double y) const;
  int toPixels(double meters) const;

  Options options_;
  double robotRadius_;
  double collisionRadius_;

  // Allocated once and cleared each frame. An earlier version allocated a
  // 3500x3500x3 Mat (~36 MB) on every iteration.
  cv::Mat canvas_;

  double scale_ = 1.0;    // px per metre
  double originX_ = 0.0;  // world coords of the canvas bottom-left
  double originY_ = 0.0;
};

}  // namespace dwa

#endif  // DWA_VISUALIZER_HPP_
