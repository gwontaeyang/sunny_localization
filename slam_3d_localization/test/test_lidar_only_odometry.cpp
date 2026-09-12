// Copyright 2026 Gole Robotics
#include "slam_3d_localization/lio_odometry.hpp"

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
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
    const Eigen::Isometry3d & base_T_sensor)
  {
    std::size_t valid_points = 0U;
    return node.prepareAndDeskewCloud(
      message, base_T_sensor, {}, Eigen::Isometry3d::Identity(),
      100000000000LL, 100000000000LL, false, valid_points);
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

}  // namespace
}  // namespace gole::slam_3d_localization
