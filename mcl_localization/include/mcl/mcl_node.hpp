#ifndef MCL_MCL_NODE_HPP_
#define MCL_MCL_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "mcl/particle_filter.hpp"

namespace mcl {

// ROS 2 front end for the filter. All ROS types stop here: the node
// translates messages into the plain structs the algorithm layer uses.
class MclNode : public rclcpp::Node {
 public:
  explicit MclNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void declareParameters();
  void buildFilter();

  void onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);
  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void onInitialPose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
  void onTimer();

  void publishEstimate(const PoseEstimate& estimate);
  void publishParticles();
  void publishMapToOdom(const PoseEstimate& estimate);

  // Reads the body -> laser transform from TF once and caches it. Falls back
  // to the configured parameters if TF is unavailable.
  void resolveLaserOffset(const std::string& laserFrame);

  // --- Parameters ---
  std::string mapFrame_;
  std::string odomFrame_;
  std::string baseFrame_;
  double updateRateHz_ = 10.0;
  double initialPoseXyStdDev_ = 0.5;
  double initialPoseYawStdDev_ = 0.2;
  bool initializeGlobally_ = false;
  bool publishTf_ = true;

  ParticleFilter::Params filterParams_;
  MotionModel::Params motionParams_;
  SensorModel::Params sensorParams_;

  // --- State ---
  std::unique_ptr<ParticleFilter> filter_;
  std::shared_ptr<const OccupancyGrid> map_;

  // Messages are held by shared pointer; callbacks no longer copy every
  // scan, odometry and (expensively) map message by value.
  nav_msgs::msg::Odometry::ConstSharedPtr lastOdom_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr lastScan_;
  bool laserOffsetResolved_ = false;
  std::optional<Pose2D> pendingInitialPose_;

  // --- ROS interfaces ---
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initialPoseSub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlePub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr posePub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mcl

#endif  // MCL_MCL_NODE_HPP_
