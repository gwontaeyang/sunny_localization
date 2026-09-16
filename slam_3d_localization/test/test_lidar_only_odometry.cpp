// Copyright 2026 Gole Robotics
#include "slam_3d_localization/lio_odometry.hpp"

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace gole::slam_3d_localization
{

// This peer lives in its own test executable, separate from the point-time peer.
class LioOdometryTestPeer
{
public:
  struct Snapshot
  {
    bool imu_subscription;
    bool wheel_subscription;
    bool processing_timer;
    bool corrected_initialized;
    bool live_initialized;
    bool have_lidar_reference;
    std::size_t imu_samples;
    std::size_t pending_clouds;
    std::size_t keyframes;
    std::size_t submap_points;
    std::size_t accepted;
    std::size_t rejected;
    std::size_t dropped;
    std::uint64_t revision;
    std::int64_t pose_stamp_ns;
    Eigen::Isometry3d pose;
    const void * submap_identity;
  };

  static Snapshot snapshot(const LioOdometry & node)
  {
    return {
      static_cast<bool>(node.imu_sub_), static_cast<bool>(node.wheel_sub_),
      static_cast<bool>(node.cloud_processing_timer_),
      node.corrected_estimator_.initialized(), node.live_estimator_.initialized(),
      node.have_lidar_reference_, node.imu_buffer_.size(), node.pending_clouds_.size(),
      node.keyframes_.size(), node.rolling_submap_->size(),
      node.accepted_registration_count_, node.rejected_registration_count_,
      node.dropped_cloud_count_, node.state_revision_, node.lidar_only_pose_stamp_ns_,
      node.lidar_only_pose_, node.rolling_submap_.get()};
  }

  static void cloud(LioOdometry & node, const sensor_msgs::msg::PointCloud2 & message)
  {
    node.cloudCallback(std::make_shared<sensor_msgs::msg::PointCloud2>(message));
  }

  static void imu(LioOdometry & node, const sensor_msgs::msg::Imu & message)
  {
    node.imuCallback(std::make_shared<sensor_msgs::msg::Imu>(message));
  }

  static void processPending(LioOdometry & node)
  {
    node.processPendingClouds();
  }

  static pcl::PointCloud<pcl::PointXYZ>::Ptr rigidCloud(
    const LioOdometry & node, const sensor_msgs::msg::PointCloud2 & message,
    const Eigen::Isometry3d & base_T_sensor, std::size_t * sampled_points = nullptr)
  {
    std::size_t valid_points = 0U;
    auto result = node.prepareAndDeskewCloud(
      message, base_T_sensor, {}, Eigen::Isometry3d::Identity(),
      100000000000LL, 100000000000LL, false, valid_points);
    if (sampled_points != nullptr) {
      *sampled_points = valid_points;
    }
    return result;
  }

  static bool timerCanceled(const LioOdometry & node)
  {
    return node.cloud_processing_timer_ && node.cloud_processing_timer_->is_canceled();
  }

  static std::size_t budgetSkipped(const LioOdometry & node)
  {
    return node.compute_budget_skipped_clouds_;
  }

  static void resetEpoch(LioOdometry & node)
  {
    node.resetForTimeEpoch("unit_test_epoch");
  }

  static bool rawCloudGroupAutoAdded(const LioOdometry & node)
  {
    return node.cloud_callback_group_->automatically_add_to_executor_with_node();
  }

  static std::size_t actualCloudDepth(const LioOdometry & node)
  {
    return node.cloud_sub_->get_actual_qos().get_rmw_qos_profile().depth;
  }

  static bool hasCloudTakeTimer(const LioOdometry & node)
  {
    return static_cast<bool>(node.cloud_take_timer_);
  }

  static void takeCloud(LioOdometry & node)
  {
    node.takeLatestCloud();
  }

  static std::size_t cloudSamplesTaken(const LioOdometry & node)
  {
    return node.cloud_samples_taken_;
  }

  static bool hasImuTakeTimer(const LioOdometry & node)
  {
    return static_cast<bool>(node.imu_take_timer_);
  }

  static bool imuGroupAutoAdded(const LioOdometry & node)
  {
    return node.imu_callback_group_ &&
           node.imu_callback_group_->automatically_add_to_executor_with_node();
  }

  static std::size_t actualImuDepth(const LioOdometry & node)
  {
    return node.imu_sub_->get_actual_qos().get_rmw_qos_profile().depth;
  }

  static void takeImu(LioOdometry & node)
  {
    node.takeImuBatch();
  }

  static bool seedImuState(LioOdometry & node, const std::int64_t stamp_ns)
  {
    ImuMeasurement measurement;
    measurement.stamp_ns = stamp_ns;
    measurement.linear_acceleration_mps2.z() = 9.80665;
    if (!node.corrected_estimator_.initialize(measurement, Eigen::Quaterniond::Identity())) {
      return false;
    }
    node.live_estimator_ = node.corrected_estimator_;
    node.latest_imu_stamp_ns_ = stamp_ns;
    return true;
  }

  static std::vector<std::int64_t> bufferedImuStamps(const LioOdometry & node)
  {
    std::vector<std::int64_t> stamps;
    for (const auto & measurement : node.imu_buffer_) {
      stamps.push_back(measurement.stamp_ns);
    }
    return stamps;
  }
};

namespace
{

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

rclcpp::NodeOptions testOptions(
  const bool imu_enabled = false, const bool wheel_enabled = false,
  const double correction_gate_m = 1.5)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("imu_enabled", imu_enabled),
    rclcpp::Parameter("wheel_correction_enabled", wheel_enabled),
    rclcpp::Parameter("publish_tf", false),
    rclcpp::Parameter("cloud_topic", "/lidar_only_unit_test/input"),
    rclcpp::Parameter("imu_topic", "/lidar_only_unit_test/imu"),
    rclcpp::Parameter("odom_topic", "/lidar_only_unit_test/odom"),
    rclcpp::Parameter("scan_odom_topic", "/lidar_only_unit_test/scan_odom"),
    rclcpp::Parameter("deskewed_cloud_topic", "/lidar_only_unit_test/points"),
    rclcpp::Parameter("diagnostics_topic", "/lidar_only_unit_test/diagnostics"),
    rclcpp::Parameter("registration_method", "gicp"),
    rclcpp::Parameter("scan_voxel_leaf_m", 0.02),
    rclcpp::Parameter("submap_voxel_leaf_m", 0.02),
    rclcpp::Parameter("min_scan_points", 20),
    rclcpp::Parameter("min_submap_points", 20),
    rclcpp::Parameter("min_range_m", 0.0),
    rclcpp::Parameter("max_range_m", 100.0),
    rclcpp::Parameter("z_min_m", -20.0),
    rclcpp::Parameter("z_max_m", 20.0),
    rclcpp::Parameter("crop_box_enabled", false),
    rclcpp::Parameter("gicp_max_iterations", 60),
    rclcpp::Parameter("max_correction_translation_m", correction_gate_m),
    rclcpp::Parameter("imu_position_in_base_m", std::vector<double>{0.1, -0.2, 1.20984})});
  return options;
}

Cloud scene()
{
  // Three nonparallel surfaces with asymmetric detail constrain all six axes.
  Cloud cloud;
  for (int u = 0; u < 13; ++u) {
    for (int v = 0; v < 11; ++v) {
      const float a = 0.27F * static_cast<float>(u);
      const float b = 0.24F * static_cast<float>(v);
      cloud.emplace_back(2.0F + a, -1.3F + b, -1.0F + 0.01F * a * b);
      cloud.emplace_back(6.0F + 0.015F * a * b, -1.7F + a, -0.8F + b);
      cloud.emplace_back(2.3F + a, 2.0F + 0.012F * a * b, -0.6F + b);
    }
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 cloudMessage(
  const std::int32_t second,
  const Eigen::Isometry3d & odom_T_base = Eigen::Isometry3d::Identity())
{
  const auto map = scene();
  Cloud scan;
  const auto base_T_odom = odom_T_base.inverse();
  for (const auto & point : map) {
    const Eigen::Vector3d transformed = base_T_odom *
      Eigen::Vector3d(point.x, point.y, point.z);
    scan.emplace_back(transformed.x(), transformed.y(), transformed.z());
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(scan, message);
  message.header.frame_id = "base_link";
  message.header.stamp.sec = second;
  message.header.stamp.nanosec = 123456789U;
  return message;
}

Eigen::Isometry3d knownMotion()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.08, -0.04, 0.06);
  pose.linear() = (
    Eigen::AngleAxisd(0.010, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(-0.008, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(0.006, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return pose;
}

class LidarOnlyOdometry : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(LidarOnlyOdometry, DefaultUsesImuWithoutBiasLearning)
{
  LioOdometry node;
  EXPECT_TRUE(node.get_parameter("imu_enabled").as_bool());
  EXPECT_FALSE(node.get_parameter("imu_bias_estimation_enabled").as_bool());
  const auto state = LioOdometryTestPeer::snapshot(node);
  EXPECT_TRUE(state.imu_subscription);
  EXPECT_FALSE(state.wheel_subscription);
  EXPECT_TRUE(state.processing_timer);
  EXPECT_FALSE(state.corrected_initialized);
  EXPECT_FALSE(state.live_initialized);
}

TEST_F(LidarOnlyOdometry, XyzOnlySeedsIdentityWithoutImuOrLeverArmCorrection)
{
  LioOdometry node(testOptions());
  const auto cloud = cloudMessage(100);
  LioOdometryTestPeer::cloud(node, cloud);
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "cv7_link";
  imu.header.stamp = cloud.header.stamp;
  imu.angular_velocity.x = 2.0;
  imu.linear_acceleration.z = 9.80665;
  LioOdometryTestPeer::imu(node, imu);

  const auto state = LioOdometryTestPeer::snapshot(node);
  ASSERT_TRUE(state.have_lidar_reference);
  EXPECT_TRUE(state.pose.matrix().isApprox(Eigen::Matrix4d::Identity(), 1.0e-12));
  EXPECT_EQ(state.pose_stamp_ns, 100123456789LL);
  EXPECT_EQ(state.keyframes, 1U);
  EXPECT_GT(state.submap_points, 20U);
  EXPECT_EQ(state.imu_samples, 0U);
  EXPECT_EQ(state.pending_clouds, 0U);
  EXPECT_FALSE(state.corrected_initialized);
  EXPECT_FALSE(state.live_initialized);
  EXPECT_EQ(state.dropped, 0U);
}

TEST_F(LidarOnlyOdometry, EstimatesKnownSixDofMotionDirectlyFromLidar)
{
  LioOdometry node(testOptions());
  LioOdometryTestPeer::cloud(node, cloudMessage(100));
  const auto expected = knownMotion();
  LioOdometryTestPeer::cloud(node, cloudMessage(101, expected));
  const auto state = LioOdometryTestPeer::snapshot(node);

  ASSERT_EQ(state.accepted, 1U);
  EXPECT_EQ(state.rejected, 0U);
  EXPECT_LT((state.pose.translation() - expected.translation()).norm(), 0.01);
  EXPECT_LT(Eigen::AngleAxisd(expected.linear().transpose() * state.pose.linear()).angle(), 0.005);
  EXPECT_GT(state.pose.translation().z(), 0.03);
  EXPECT_FALSE(state.corrected_initialized);
  EXPECT_FALSE(state.live_initialized);
}

TEST_F(LidarOnlyOdometry, RejectedRegistrationPreservesAcceptedPoseAndSubmap)
{
  LioOdometry node(testOptions(false, false, 0.005));
  LioOdometryTestPeer::cloud(node, cloudMessage(100));
  const auto before = LioOdometryTestPeer::snapshot(node);
  LioOdometryTestPeer::cloud(node, cloudMessage(101, knownMotion()));
  const auto after = LioOdometryTestPeer::snapshot(node);

  ASSERT_EQ(after.rejected, before.rejected + 1U);
  EXPECT_EQ(after.accepted, before.accepted);
  EXPECT_EQ(after.pose_stamp_ns, before.pose_stamp_ns);
  EXPECT_TRUE(after.pose.matrix().isApprox(before.pose.matrix(), 1.0e-12));
  EXPECT_EQ(after.keyframes, before.keyframes);
  EXPECT_EQ(after.submap_identity, before.submap_identity);
}

TEST_F(LidarOnlyOdometry, InvalidCloudsCannotSeedState)
{
  LioOdometry node(testOptions());
  auto bigendian = cloudMessage(100);
  bigendian.is_bigendian = true;
  LioOdometryTestPeer::cloud(node, bigendian);

  auto truncated = cloudMessage(101);
  truncated.data.pop_back();
  LioOdometryTestPeer::cloud(node, truncated);

  auto short_row = cloudMessage(102);
  short_row.row_step = short_row.point_step * (short_row.width - 1U);
  LioOdometryTestPeer::cloud(node, short_row);

  auto missing_frame = cloudMessage(103);
  missing_frame.header.frame_id.clear();
  LioOdometryTestPeer::cloud(node, missing_frame);

  const auto state = LioOdometryTestPeer::snapshot(node);
  EXPECT_FALSE(state.have_lidar_reference);
  EXPECT_EQ(state.keyframes, 0U);
  EXPECT_EQ(state.submap_points, 0U);
  EXPECT_EQ(state.pose_stamp_ns, -1);
  EXPECT_EQ(state.dropped, 4U);
}

TEST_F(LidarOnlyOdometry, DuplicateCloudDoesNotRegisterAgain)
{
  LioOdometry node(testOptions());
  const auto cloud = cloudMessage(100);
  LioOdometryTestPeer::cloud(node, cloud);
  LioOdometryTestPeer::cloud(node, cloud);
  const auto state = LioOdometryTestPeer::snapshot(node);
  EXPECT_EQ(state.accepted, 0U);
  EXPECT_EQ(state.keyframes, 1U);
  EXPECT_EQ(state.dropped, 1U);
  EXPECT_EQ(state.pose_stamp_ns, 100123456789LL);
}

TEST_F(LidarOnlyOdometry, CloudTimestampRewindStartsNewIdentityReference)
{
  LioOdometry node(testOptions());
  LioOdometryTestPeer::cloud(node, cloudMessage(100));
  LioOdometryTestPeer::cloud(node, cloudMessage(101, knownMotion()));
  const auto before = LioOdometryTestPeer::snapshot(node);
  ASSERT_EQ(before.accepted, 1U);

  LioOdometryTestPeer::cloud(node, cloudMessage(10, knownMotion()));
  const auto after = LioOdometryTestPeer::snapshot(node);
  EXPECT_TRUE(after.have_lidar_reference);
  EXPECT_TRUE(after.pose.matrix().isApprox(Eigen::Matrix4d::Identity(), 1.0e-12));
  EXPECT_EQ(after.pose_stamp_ns, 10123456789LL);
  EXPECT_EQ(after.keyframes, 1U);
  EXPECT_GT(after.revision, before.revision);
  EXPECT_FALSE(after.corrected_initialized);
}

TEST_F(LidarOnlyOdometry, StaticSensorTransformIsStillApplied)
{
  LioOdometry node(testOptions());
  Cloud cloud;
  cloud.emplace_back(2.0F, 1.0F, -0.5F);
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(1.0, -0.2, 0.4);
  transform.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto result = LioOdometryTestPeer::rigidCloud(node, message, transform);
  ASSERT_EQ(result->size(), 1U);
  const Eigen::Vector3d expected = transform * Eigen::Vector3d(2.0, 1.0, -0.5);
  EXPECT_NEAR(result->front().x, expected.x(), 1.0e-6);
  EXPECT_NEAR(result->front().y, expected.y(), 1.0e-6);
  EXPECT_NEAR(result->front().z, expected.z(), 1.0e-6);
}

TEST_F(LidarOnlyOdometry, RawPointWorkIsCappedBeforeVoxelFiltering)
{
  LioOdometry node(testOptions());
  Cloud cloud;
  for (std::size_t index = 0U; index < 8192U; ++index) {
    cloud.emplace_back(
      2.0F + 0.05F * static_cast<float>(index % 128U),
      0.05F * static_cast<float>(index / 128U), 0.0F);
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  std::size_t sampled = 0U;
  const auto result = LioOdometryTestPeer::rigidCloud(
    node, message, Eigen::Isometry3d::Identity(), &sampled);
  EXPECT_EQ(sampled, 1024U);
  EXPECT_EQ(result->size(), 1024U);
}

TEST_F(LidarOnlyOdometry, SubsamplingRespectsOrganizedRowPadding)
{
  LioOdometry node(testOptions());
  Cloud cloud;
  for (std::size_t index = 0U; index < 2048U; ++index) {
    cloud.emplace_back(
      2.0F + 0.05F * static_cast<float>(index % 64U),
      0.05F * static_cast<float>(index / 64U), 0.0F);
  }
  sensor_msgs::msg::PointCloud2 packed;
  pcl::toROSMsg(cloud, packed);
  auto padded = packed;
  padded.width = 64U;
  padded.height = 32U;
  padded.row_step = padded.width * padded.point_step + 13U;
  padded.data.assign(static_cast<std::size_t>(padded.row_step) * padded.height, 0xFFU);
  for (std::size_t row = 0U; row < padded.height; ++row) {
    std::memcpy(
      padded.data.data() + row * padded.row_step,
      packed.data.data() + row * padded.width * padded.point_step,
      static_cast<std::size_t>(padded.width) * padded.point_step);
  }
  std::size_t sampled = 0U;
  const auto actual = LioOdometryTestPeer::rigidCloud(
    node, padded, Eigen::Isometry3d::Identity(), &sampled);
  const auto expected = LioOdometryTestPeer::rigidCloud(
    node, packed, Eigen::Isometry3d::Identity());
  EXPECT_EQ(sampled, 1024U);
  ASSERT_EQ(actual->size(), expected->size());
  ASSERT_EQ(actual->size(), 1024U);
  for (std::size_t index = 0U; index < actual->size(); ++index) {
    EXPECT_FLOAT_EQ((*actual)[index].x, (*expected)[index].x);
    EXPECT_FLOAT_EQ((*actual)[index].y, (*expected)[index].y);
    EXPECT_FLOAT_EQ((*actual)[index].z, (*expected)[index].z);
  }
}

TEST_F(LidarOnlyOdometry, PendingTimerIsDemandDrivenAndResetCancelsIt)
{
  auto options = testOptions(true);
  auto parameters = options.parameter_overrides();
  parameters.emplace_back("require_point_time", false);
  options.parameter_overrides(parameters);
  LioOdometry node(options);
  EXPECT_TRUE(LioOdometryTestPeer::timerCanceled(node));
  LioOdometryTestPeer::cloud(node, cloudMessage(100));
  EXPECT_FALSE(LioOdometryTestPeer::timerCanceled(node));
  ASSERT_EQ(LioOdometryTestPeer::snapshot(node).pending_clouds, 1U);
  LioOdometryTestPeer::resetEpoch(node);
  EXPECT_TRUE(LioOdometryTestPeer::timerCanceled(node));
  EXPECT_EQ(LioOdometryTestPeer::snapshot(node).pending_clouds, 0U);
}

TEST_F(LidarOnlyOdometry, ImuModeUsesManualLatestOnlyDdsTake)
{
  auto options = testOptions(true);
  auto parameters = options.parameter_overrides();
  parameters.emplace_back("cloud_subscription_depth", 20);
  options.parameter_overrides(parameters);
  LioOdometry node(options);
  EXPECT_FALSE(LioOdometryTestPeer::rawCloudGroupAutoAdded(node));
  EXPECT_EQ(LioOdometryTestPeer::actualCloudDepth(node), 1U);
  EXPECT_TRUE(LioOdometryTestPeer::hasCloudTakeTimer(node));
  EXPECT_EQ(node.get_parameter("cloud_subscription_depth").as_int(), 20);
  EXPECT_NO_THROW(LioOdometryTestPeer::takeCloud(node));
  EXPECT_EQ(LioOdometryTestPeer::cloudSamplesTaken(node), 0U);
  EXPECT_EQ(LioOdometryTestPeer::snapshot(node).pending_clouds, 0U);
}

TEST_F(LidarOnlyOdometry, LidarOnlyRetainsAutomaticCallbackAndRequestedDdsDepth)
{
  auto options = testOptions(false);
  auto parameters = options.parameter_overrides();
  parameters.emplace_back("cloud_subscription_depth", 7);
  options.parameter_overrides(parameters);
  LioOdometry node(options);
  EXPECT_TRUE(LioOdometryTestPeer::rawCloudGroupAutoAdded(node));
  EXPECT_EQ(LioOdometryTestPeer::actualCloudDepth(node), 7U);
  EXPECT_FALSE(LioOdometryTestPeer::hasCloudTakeTimer(node));
  EXPECT_FALSE(LioOdometryTestPeer::snapshot(node).processing_timer);
}

TEST_F(LidarOnlyOdometry, ImuBatchTimerUsesManualTakeAndRetainsFullQueue)
{
  LioOdometry node(testOptions(true));
  EXPECT_TRUE(LioOdometryTestPeer::hasImuTakeTimer(node));
  EXPECT_FALSE(LioOdometryTestPeer::imuGroupAutoAdded(node));
  EXPECT_EQ(LioOdometryTestPeer::actualImuDepth(node), 400U);
  EXPECT_NO_THROW(LioOdometryTestPeer::takeImu(node));
  const auto state = LioOdometryTestPeer::snapshot(node);
  EXPECT_FALSE(state.corrected_initialized);
  EXPECT_FALSE(state.live_initialized);
  EXPECT_EQ(state.imu_samples, 0U);
}

TEST_F(LidarOnlyOdometry, DisabledImuHasNoBatchTimerOrSubscription)
{
  LioOdometry node(testOptions(false));
  EXPECT_FALSE(LioOdometryTestPeer::hasImuTakeTimer(node));
  EXPECT_FALSE(LioOdometryTestPeer::snapshot(node).imu_subscription);
}

TEST_F(LidarOnlyOdometry, ImuBatchDdsDeliveryPreservesBothOriginalTimestamps)
{
  // No robot inputs or executor spin: two synthetic loopback messages, bounded
  // discovery/delivery waits, then the production manual-take method.
  LioOdometry node(testOptions(true));
  ASSERT_TRUE(LioOdometryTestPeer::seedImuState(node, 1000000000LL));
  auto sender = std::make_shared<rclcpp::Node>("imu_batch_unit_test_sender");
  auto publisher = sender->create_publisher<sensor_msgs::msg::Imu>(
    node.get_parameter("imu_topic").as_string(), rclcpp::SensorDataQoS().keep_last(10));
  const auto discovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (publisher->get_subscription_count() == 0U &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0U);
  sensor_msgs::msg::Imu message;
  message.header.frame_id = node.get_parameter("imu_frame").as_string();
  message.header.stamp.sec = 1;
  message.linear_acceleration.z = 9.80665;
  message.header.stamp.nanosec = 5000000U;
  publisher->publish(message);
  message.header.stamp.nanosec = 10000000U;
  publisher->publish(message);
  const auto delivery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (LioOdometryTestPeer::bufferedImuStamps(node).size() < 2U &&
    std::chrono::steady_clock::now() < delivery_deadline)
  {
    LioOdometryTestPeer::takeImu(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  const auto stamps = LioOdometryTestPeer::bufferedImuStamps(node);
  ASSERT_EQ(stamps.size(), 2U);
  EXPECT_EQ(stamps[0], 1005000000LL);
  EXPECT_EQ(stamps[1], 1010000000LL);
}

TEST_F(LidarOnlyOdometry, ImuCloudCallbackAppliesHalfHertzBudget)
{
  auto options = testOptions(true);
  auto parameters = options.parameter_overrides();
  parameters.emplace_back("require_point_time", false);
  options.parameter_overrides(parameters);
  LioOdometry node(options);
  auto message = cloudMessage(100);
  LioOdometryTestPeer::cloud(node, message);
  message.header.stamp.nanosec += 100000000U;
  LioOdometryTestPeer::cloud(node, message);
  EXPECT_EQ(LioOdometryTestPeer::budgetSkipped(node), 1U);
  EXPECT_EQ(LioOdometryTestPeer::snapshot(node).pending_clouds, 1U);
  LioOdometryTestPeer::cloud(node, cloudMessage(101));
  EXPECT_EQ(LioOdometryTestPeer::budgetSkipped(node), 2U);
  EXPECT_EQ(LioOdometryTestPeer::snapshot(node).pending_clouds, 1U);
  LioOdometryTestPeer::cloud(node, cloudMessage(102));
  EXPECT_EQ(LioOdometryTestPeer::snapshot(node).pending_clouds, 2U);
  LioOdometryTestPeer::resetEpoch(node);
  LioOdometryTestPeer::cloud(node, cloudMessage(10));
  EXPECT_EQ(LioOdometryTestPeer::snapshot(node).pending_clouds, 1U);
  EXPECT_FALSE(LioOdometryTestPeer::timerCanceled(node));
}

TEST_F(LidarOnlyOdometry, ExplicitImuModeRetainsInitializationGate)
{
  auto options = testOptions(true);
  auto parameters = options.parameter_overrides();
  parameters.emplace_back("require_point_time", false);
  options.parameter_overrides(parameters);
  LioOdometry node(options);
  LioOdometryTestPeer::cloud(node, cloudMessage(100));
  LioOdometryTestPeer::processPending(node);
  const auto state = LioOdometryTestPeer::snapshot(node);
  EXPECT_TRUE(state.imu_subscription);
  EXPECT_TRUE(state.processing_timer);
  EXPECT_FALSE(state.wheel_subscription);
  EXPECT_FALSE(state.have_lidar_reference);
  EXPECT_FALSE(state.corrected_initialized);
  EXPECT_EQ(state.pending_clouds, 1U);
  EXPECT_EQ(state.keyframes, 0U);
}

TEST_F(LidarOnlyOdometry, WheelAidingWithoutImuFailsExplicitly)
{
  EXPECT_THROW(LioOdometry node(testOptions(false, true)), std::invalid_argument);
}

TEST_F(LidarOnlyOdometry, RejectsMinimumScanPointsAboveFixedPointCap)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "min_scan_points", static_cast<int>(LowComputeBudget::kMaxDeskewPoints) + 1)});
  try {
    LioOdometry node(options);
    FAIL() << "min_scan_points above the hard cap must fail at startup";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("min_scan_points"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find(
        std::to_string(LowComputeBudget::kMaxDeskewPoints)), std::string::npos);
  }
}

TEST_F(LidarOnlyOdometry, RejectsMinimumSubmapPointsAboveFixedPointCap)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter(
      "min_submap_points", static_cast<int>(LowComputeBudget::kMaxDeskewPoints) + 1)});
  try {
    LioOdometry node(options);
    FAIL() << "min_submap_points above the hard cap must fail at startup";
  } catch (const std::invalid_argument & error) {
    EXPECT_NE(std::string(error.what()).find("min_submap_points"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find(
        std::to_string(LowComputeBudget::kMaxDeskewPoints)), std::string::npos);
  }
}

}  // namespace
}  // namespace gole::slam_3d_localization
