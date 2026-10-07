#include "dwa/visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace dwa {
namespace {

const cv::Scalar kWhite(255, 255, 255);
const cv::Scalar kBlack(0, 0, 0);
const cv::Scalar kRed(0, 0, 255);
const cv::Scalar kGreen(0, 200, 0);
const cv::Scalar kGrey(190, 190, 190);
const cv::Scalar kBlue(200, 120, 0);

}  // namespace

Visualizer::Visualizer(Options options, const std::vector<Obstacle>& obstacles,
                       const Eigen::Vector2d& start, const Eigen::Vector2d& goal,
                       double robotRadius, double collisionRadius)
    : options_(std::move(options)),
      robotRadius_(robotRadius),
      collisionRadius_(collisionRadius) {
  // Fit the view to everything we know about up front.
  double minX = std::min(start.x(), goal.x());
  double maxX = std::max(start.x(), goal.x());
  double minY = std::min(start.y(), goal.y());
  double maxY = std::max(start.y(), goal.y());

  for (const Obstacle& obstacle : obstacles) {
    minX = std::min(minX, obstacle.x);
    maxX = std::max(maxX, obstacle.x);
    minY = std::min(minY, obstacle.y);
    maxY = std::max(maxY, obstacle.y);
  }

  const double margin = options_.marginMeters + collisionRadius_;
  minX -= margin;
  maxX += margin;
  minY -= margin;
  maxY += margin;

  // Keep the aspect ratio square so circles stay circular.
  const double extent = std::max(maxX - minX, maxY - minY);
  const double safeExtent = extent > 1e-9 ? extent : 1.0;

  originX_ = 0.5 * (minX + maxX) - 0.5 * safeExtent;
  originY_ = 0.5 * (minY + maxY) - 0.5 * safeExtent;
  scale_ = static_cast<double>(options_.canvasSize) / safeExtent;

  canvas_.create(options_.canvasSize, options_.canvasSize, CV_8UC3);
  cv::namedWindow(options_.windowName, cv::WINDOW_NORMAL);
  cv::resizeWindow(options_.windowName, options_.canvasSize, options_.canvasSize);
}

Visualizer::~Visualizer() {
  cv::destroyWindow(options_.windowName);
}

cv::Point Visualizer::toPixel(double x, double y) const {
  const int px = static_cast<int>(std::lround((x - originX_) * scale_));
  // Flip y: world y grows upward, image rows grow downward.
  const int py = static_cast<int>(
      std::lround(static_cast<double>(options_.canvasSize) - (y - originY_) * scale_));
  return cv::Point(px, py);
}

int Visualizer::toPixels(double meters) const {
  return std::max(1, static_cast<int>(std::lround(meters * scale_)));
}

void Visualizer::render(const State& state, const Trajectory& trajectory,
                        const std::vector<Obstacle>& obstacles,
                        const Eigen::Vector2d& goal, bool recovering) {
  canvas_.setTo(kWhite);

  // Goal tolerance marker.
  cv::circle(canvas_, toPixel(goal.x(), goal.y()), toPixels(0.6), kGreen, -1);

  // Obstacles, each with its collision boundary so the margin is visible.
  for (const Obstacle& obstacle : obstacles) {
    const cv::Point center = toPixel(obstacle.x, obstacle.y);
    cv::circle(canvas_, center, toPixels(collisionRadius_), kGrey, 1);
    cv::circle(canvas_, center, toPixels(std::max(0.25, collisionRadius_ - robotRadius_)),
               kBlack, -1);
  }

  // Chosen trajectory.
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    cv::line(canvas_, toPixel(trajectory[i - 1](kX), trajectory[i - 1](kY)),
             toPixel(trajectory[i](kX), trajectory[i](kY)), kRed, 2);
  }

  // Robot footprint and heading.
  const cv::Point robot = toPixel(state(kX), state(kY));
  cv::circle(canvas_, robot, toPixels(robotRadius_), recovering ? kBlue : kBlack, 2);
  cv::arrowedLine(canvas_, robot,
                  toPixel(state(kX) + robotRadius_ * std::cos(state(kTheta)),
                          state(kY) + robotRadius_ * std::sin(state(kTheta))),
                  recovering ? kBlue : kBlack, 2);

  if (recovering) {
    cv::putText(canvas_, "RECOVERY: no collision-free trajectory",
                cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.6, kBlue, 2);
  }
}

int Visualizer::show(int waitMs) {
  cv::imshow(options_.windowName, canvas_);
  return cv::waitKey(waitMs);
}

}  // namespace dwa
