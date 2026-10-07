#include "mcl/mcl_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace mcl {
namespace {

// Yaw from a quaternion, and back. Done by hand rather than through
// tf2::getYaw so this file does not depend on which spelling of the tf2
// utility headers the distribution ships (tf2/utils.h vs tf2/utils.hpp).
double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

Pose2D poseFromMsg(const geometry_msgs::msg::Pose& pose) {
  return Pose2D(pose.position.x, pose.position.y,
                yawFromQuaternion(pose.orientation));
}

// Inverse of a planar rigid transform.
Pose2D inverse(const Pose2D& pose) {
  const double c = std::cos(pose.theta);
  const double s = std::sin(pose.theta);
  return Pose2D(-(pose.x * c + pose.y * s), pose.x * s - pose.y * c,
                wrapToPi(-pose.theta));
}

}  // namespace

MclNode::MclNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mcl_localization", options) {
  declareParameters();
  buildFilter();

  tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_, this);

  // The map is published latched; transient_local durability is what makes a
  // late subscriber receive it.
  const rclcpp::QoS mapQos = rclcpp::QoS(1).transient_local().reliable();

  mapSub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", mapQos,
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { onMap(msg); });

  scanSub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { onScan(msg); });

  odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdom(msg); });

  initialPoseSub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        onInitialPose(msg);
      });

  particlePub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud",
                                                                rclcpp::QoS(1));
  posePub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "mcl_pose", rclcpp::QoS(1));

  // A wall timer replaces the hand-rolled while(ros::ok()) loop, and spinning
  // happens in exactly one place (the executor) rather than in both main and
  // the update function.
  const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, updateRateHz_));
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { onTimer(); });

  RCLCPP_INFO(get_logger(),
              "mcl_localization up: %d particles, every %dth beam, sigma_hit %.2f m",
              filterParams_.particleCount, sensorParams_.beamSkip,
              sensorParams_.sigmaHit);
}

void MclNode::declareParameters() {
  // Every tuning value is a declared parameter. At first these lived
  // in two places -- the filter header and the model header -- and the
  // model's constructor silently discarded the ones it was handed, so the
  // header defaults were what actually ran and tuning was a no-op.
  mapFrame_ = declare_parameter<std::string>("map_frame", "map");
  odomFrame_ = declare_parameter<std::string>("odom_frame", "odom");
  baseFrame_ = declare_parameter<std::string>("base_frame", "base_link");
  updateRateHz_ = declare_parameter<double>("update_rate_hz", 10.0);
  publishTf_ = declare_parameter<bool>("publish_tf", true);

  initializeGlobally_ = declare_parameter<bool>("initialize_globally", false);
  initialPoseXyStdDev_ = declare_parameter<double>("initial_pose_xy_stddev", 0.5);
  initialPoseYawStdDev_ = declare_parameter<double>("initial_pose_yaw_stddev", 0.2);

  filterParams_.particleCount = declare_parameter<int>("particle_count", 1000);
  filterParams_.minTranslationForUpdate =
      declare_parameter<double>("min_translation_for_update", 0.02);
  filterParams_.minRotationForUpdate =
      declare_parameter<double>("min_rotation_for_update", 0.02);
  filterParams_.resampleThresholdRatio =
      declare_parameter<double>("resample_threshold_ratio", 0.5);
  filterParams_.seed =
      static_cast<std::uint32_t>(declare_parameter<int>("random_seed", 42));

  motionParams_.alpha1 = declare_parameter<double>("alpha1", 0.05);
  motionParams_.alpha2 = declare_parameter<double>("alpha2", 0.05);
  motionParams_.alpha3 = declare_parameter<double>("alpha3", 0.05);
  motionParams_.alpha4 = declare_parameter<double>("alpha4", 0.05);
  motionParams_.minTranslation = declare_parameter<double>("motion_min_translation", 1e-3);

  sensorParams_.zHit = declare_parameter<double>("z_hit", 0.9);
  sensorParams_.zRand = declare_parameter<double>("z_rand", 0.1);
  sensorParams_.sigmaHit = declare_parameter<double>("sigma_hit", 0.1);
  sensorParams_.beamSkip = declare_parameter<int>("beam_skip", 12);
  sensorParams_.maxRange = declare_parameter<double>("laser_max_range", 0.0);
  sensorParams_.normalizeByBeamCount =
      declare_parameter<bool>("normalize_by_beam_count", true);

  // Fallback laser offset, used only if the TF lookup fails.
  filterParams_.laserOffset =
      Pose2D(declare_parameter<double>("laser_offset_x", 0.0),
             declare_parameter<double>("laser_offset_y", 0.0),
             declare_parameter<double>("laser_offset_yaw", 0.0));
}

void MclNode::buildFilter() {
  filter_ = std::make_unique<ParticleFilter>(filterParams_, motionParams_,
                                             sensorParams_);
}

void MclNode::onMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg) {
  if (msg->info.width == 0 || msg->info.height == 0) {
    RCLCPP_WARN(get_logger(), "ignoring empty map");
    return;
  }

  // msg->info.origin is honoured -- an earlier version never read it, so every
  // world/grid conversion silently assumed an origin of (0, 0, 0).
  const Pose2D origin = poseFromMsg(msg->info.origin);

  std::vector<int8_t> cells(msg->data.begin(), msg->data.end());
  auto grid = std::make_shared<OccupancyGrid>(
      static_cast<int>(msg->info.width), static_cast<int>(msg->info.height),
      msg->info.resolution, origin, std::move(cells));

  if (grid->empty()) {
    RCLCPP_ERROR(get_logger(), "map data size does not match its declared extent");
    return;
  }

  map_ = grid;
  // setMap clears all derived state, so a republished (latched) map replaces
  // the old one instead of being appended to it.
  filter_->setMap(map_);

  RCLCPP_INFO(get_logger(), "map: %dx%d at %.3f m/cell, origin (%.2f, %.2f, %.2f rad)",
              grid->width(), grid->height(), grid->resolution(), origin.x, origin.y,
              origin.theta);

  if (pendingInitialPose_) {
    filter_->initializeAtPose(*pendingInitialPose_, initialPoseXyStdDev_,
                              initialPoseYawStdDev_);
    pendingInitialPose_.reset();
    RCLCPP_INFO(get_logger(), "initialized from the pending initial pose");
  } else if (initializeGlobally_) {
    if (filter_->initializeGlobal()) {
      RCLCPP_INFO(get_logger(), "initialized globally over the map free space");
    } else {
      RCLCPP_ERROR(get_logger(), "global initialization found no free space");
    }
  } else {
    RCLCPP_INFO(get_logger(),
                "waiting for an initial pose on 'initialpose' "
                "(set initialize_globally:=true to start from a uniform cloud)");
  }
}

void MclNode::onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  lastScan_ = msg;
  if (!laserOffsetResolved_) {
    resolveLaserOffset(msg->header.frame_id);
  }
}

void MclNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  lastOdom_ = msg;
}

void MclNode::onInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  const Pose2D pose = poseFromMsg(msg->pose.pose);

  if (!filter_->hasMap()) {
    // Hold it until the map arrives rather than dropping it.
    pendingInitialPose_ = pose;
    RCLCPP_INFO(get_logger(), "initial pose stored; waiting for a map");
    return;
  }

  filter_->initializeAtPose(pose, initialPoseXyStdDev_, initialPoseYawStdDev_);
  RCLCPP_INFO(get_logger(), "re-initialized at (%.2f, %.2f, %.2f rad)", pose.x,
              pose.y, pose.theta);
}

void MclNode::resolveLaserOffset(const std::string& laserFrame) {
  if (laserFrame.empty() || laserFrame == baseFrame_) {
    laserOffsetResolved_ = true;
    return;
  }

  try {
    const geometry_msgs::msg::TransformStamped tf = tfBuffer_->lookupTransform(
        baseFrame_, laserFrame, tf2::TimePointZero);

    const Pose2D offset(tf.transform.translation.x, tf.transform.translation.y,
                        yawFromQuaternion(tf.transform.rotation));
    filterParams_.laserOffset = offset;
    // Applied in place: TF often resolves after the map has arrived and the
    // cloud has been initialized, and rebuilding the filter here would
    // discard it.
    filter_->setLaserOffset(offset);
    laserOffsetResolved_ = true;

    RCLCPP_INFO(get_logger(), "laser offset from TF %s->%s: (%.3f, %.3f, %.3f rad)",
                baseFrame_.c_str(), laserFrame.c_str(), offset.x, offset.y,
                offset.theta);
  } catch (const tf2::TransformException& ex) {
    // Not fatal: retried on the next scan, and the parameter value stands in
    // the meantime.
    RCLCPP_DEBUG(get_logger(), "waiting for TF %s->%s: %s", baseFrame_.c_str(),
                 laserFrame.c_str(), ex.what());
  }
}

void MclNode::onTimer() {
  if (!filter_->initialized() || !lastOdom_ || !lastScan_) {
    return;
  }

  LaserScan scan;
  scan.angleMin = lastScan_->angle_min;
  scan.angleIncrement = lastScan_->angle_increment;
  scan.rangeMin = lastScan_->range_min;
  scan.rangeMax = lastScan_->range_max;
  scan.ranges = lastScan_->ranges;

  const Pose2D odom = poseFromMsg(lastOdom_->pose.pose);

  if (!filter_->update(odom, scan)) {
    // Below the motion threshold, or still establishing the odometry
    // reference. Keep republishing so RViz does not show a stale cloud.
    publishParticles();
    return;
  }

  const PoseEstimate estimate = filter_->estimate();
  if (!estimate.valid) {
    return;
  }

  publishEstimate(estimate);
  publishParticles();
  if (publishTf_) {
    publishMapToOdom(estimate);
  }
}

void MclNode::publishEstimate(const PoseEstimate& estimate) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = now();
  msg.header.frame_id = mapFrame_;
  msg.pose.pose.position.x = estimate.mean.x;
  msg.pose.pose.position.y = estimate.mean.y;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = quaternionFromYaw(estimate.mean.theta);

  // 6x6 row-major ROS covariance from our 3x3 (x, y, yaw) block.
  msg.pose.covariance.fill(0.0);
  msg.pose.covariance[0] = estimate.covariance[0];   // xx
  msg.pose.covariance[1] = estimate.covariance[1];   // xy
  msg.pose.covariance[6] = estimate.covariance[3];   // yx
  msg.pose.covariance[7] = estimate.covariance[4];   // yy
  msg.pose.covariance[35] = estimate.covariance[8];  // yaw-yaw

  posePub_->publish(msg);
}

void MclNode::publishParticles() {
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = now();
  msg.header.frame_id = mapFrame_;

  const std::vector<Particle>& particles = filter_->particles();
  msg.poses.reserve(particles.size());
  for (const Particle& particle : particles) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = particle.pose.x;
    pose.position.y = particle.pose.y;
    pose.position.z = 0.0;
    pose.orientation = quaternionFromYaw(particle.pose.theta);
    msg.poses.push_back(pose);
  }
  particlePub_->publish(msg);
}

void MclNode::publishMapToOdom(const PoseEstimate& estimate) {
  if (!lastOdom_) {
    return;
  }

  // A localizer publishes map -> odom; the odometry source owns
  // odom -> base_link.
  //
  // An earlier version broadcast map -> base_link directly, which gives base_link
  // two parents the moment any odometry publisher is running and breaks the
  // TF tree. Its frame IDs also carried leading slashes, which tf2 rejects.
  const Pose2D odom = poseFromMsg(lastOdom_->pose.pose);
  const Pose2D mapToOdom = compose(estimate.mean, inverse(odom));

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = now();
  tf.header.frame_id = mapFrame_;
  tf.child_frame_id = odomFrame_;
  tf.transform.translation.x = mapToOdom.x;
  tf.transform.translation.y = mapToOdom.y;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation = quaternionFromYaw(mapToOdom.theta);

  tfBroadcaster_->sendTransform(tf);
}

}  // namespace mcl
