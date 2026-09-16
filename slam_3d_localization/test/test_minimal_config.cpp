// Copyright 2026 Gole Robotics
#include "slam_3d_localization/lio_odometry.hpp"
#include "slam_3d_localization/slam_3d_localizer.hpp"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace gole::slam_3d_localization
{

class Slam3dLocalizerTestPeer
{
public:
  static bool timerCanceled(const Slam3dLocalizer & node)
  {
    return node.pending_cloud_timer_->is_canceled();
  }

  static std::size_t pendingCount(const Slam3dLocalizer & node)
  {
    return node.pending_clouds_.size();
  }

  static void cloud(Slam3dLocalizer & node, const std::int32_t seconds)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.emplace_back(5.0F, 0.0F, 0.0F);
    auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud, *message);
    message->header.frame_id = "base_link";
    message->header.stamp.sec = seconds;
    node.cloudCallback(message);
  }

  static void exactOdom(Slam3dLocalizer & node, const std::int32_t seconds)
  {
    auto message = std::make_shared<nav_msgs::msg::Odometry>();
    message->header.frame_id = "odom";
    message->child_frame_id = "base_link";
    message->header.stamp.sec = seconds;
    message->pose.pose.orientation.w = 1.0;
    node.registrationOdomCallback(message);
  }

  static void retry(Slam3dLocalizer & node)
  {
    node.processPendingClouds();
  }

  static void initialPose(Slam3dLocalizer & node, const std::string & frame = "map")
  {
    auto message = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    message->header.frame_id = frame;
    message->pose.pose.orientation.w = 1.0;
    node.initialPoseCallback(message);
  }

  static void seedLastCloudStamp(Slam3dLocalizer & node, const std::int32_t seconds)
  {
    node.have_last_cloud_stamp_ = true;
    node.last_cloud_stamp_ns_ = static_cast<std::int64_t>(seconds) * 1000000000LL;
  }
};

namespace
{

using ParameterSnapshot = std::map<std::string, rclcpp::Parameter>;

class MinimalConfigTest : public ::testing::Test
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

  void SetUp() override
  {
    // Synthetic input only: exercise map loading without depending on a robot,
    // a private map, a bag, a display, or any hardware bringup.
    const std::string pattern =
      (std::filesystem::temp_directory_path() / "localization_config_XXXXXX").string();
    std::vector<char> path(pattern.begin(), pattern.end());
    path.push_back('\0');
    const int descriptor = mkstemp(path.data());
    ASSERT_GE(descriptor, 0);
    close(descriptor);
    map_path_ = path.data();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int x = 0; x < 8; ++x) {
      for (int y = 0; y < 8; ++y) {
        for (int z = 0; z < 8; ++z) {
          cloud.emplace_back(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
        }
      }
    }
    ASSERT_EQ(pcl::io::savePCDFileBinary(map_path_, cloud), 0);
  }

  void TearDown() override
  {
    if (!map_path_.empty()) {
      std::filesystem::remove(map_path_);
    }
  }

  template<typename NodeT>
  ParameterSnapshot snapshotFiles(
    const std::vector<std::string> & files,
    const std::vector<rclcpp::Parameter> & runtime_overrides = {})
  {
    std::vector<std::string> arguments{"--ros-args"};
    for (const auto & file : files) {
      arguments.insert(arguments.end(), {"--params-file", file});
    }
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.arguments(arguments);
    // The map location is an operational input, not an algorithm default.
    // Runtime launch arguments intentionally take precedence over YAML files.
    auto overrides = runtime_overrides;
    overrides.emplace_back("map_path", map_path_);
    options.parameter_overrides(overrides);
    auto node = std::make_shared<NodeT>(options);
    const auto names = node->list_parameters({}, 0).names;
    ParameterSnapshot parameters;
    for (const auto & parameter : node->get_parameters(names)) {
      parameters.emplace(parameter.get_name(), parameter);
    }
    return parameters;
  }

  std::shared_ptr<Slam3dLocalizer> timerTestNode(const bool exact = true)
  {
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({
      rclcpp::Parameter("map_path", map_path_),
      rclcpp::Parameter("publish_global_map", false),
      rclcpp::Parameter("publish_tf", false),
      rclcpp::Parameter("require_exact_odom_for_update", exact),
      rclcpp::Parameter("require_synced_odom_for_update", exact),
      rclcpp::Parameter("registration_odom_topic", "/minimal_config/scan_odom")});
    return std::make_shared<Slam3dLocalizer>(options);
  }

  template<typename NodeT>
  ParameterSnapshot snapshot(const std::string & filename, const bool legacy)
  {
    const std::filesystem::path root(SLAM_3D_LOCALIZATION_SOURCE_DIR);
    const auto profile = legacy ?
      root / "test" / "fixtures" / "legacy_config" / filename :
      root / "config" / filename;
    std::vector<std::string> files;
    if (!legacy && filename == "slam_3d_lio_gangnam.yaml") {
      // Generated from the same method used by the launch file, not a second
      // handwritten copy of its internal topic/synchronization defaults.
      files.emplace_back(SLAM_3D_LOCALIZATION_LIO_PROFILE_FILE);
    }
    files.push_back(profile.string());
    return snapshotFiles<NodeT>(files);
  }

  template<typename NodeT>
  void expectSameEffectiveParameters(
    const std::string & filename,
    const std::vector<rclcpp::Parameter> & intentional_changes = {})
  {
    auto legacy = snapshot<NodeT>(filename, true);
    for (const auto & change : intentional_changes) {
      ASSERT_NE(legacy.find(change.get_name()), legacy.end());
      legacy.at(change.get_name()) = change;
    }
    const auto minimal = snapshot<NodeT>(filename, false);
    ASSERT_FALSE(legacy.empty());
    ASSERT_EQ(minimal.size(), legacy.size()) << filename;
    for (const auto & entry : legacy) {
      SCOPED_TRACE(filename + ": " + entry.first);
      const auto actual = minimal.find(entry.first);
      ASSERT_NE(actual, minimal.end());
      EXPECT_EQ(actual->second.get_type(), entry.second.get_type());
      EXPECT_EQ(actual->second.get_parameter_value(), entry.second.get_parameter_value());
    }
  }

  std::string map_path_;
};

TEST_F(MinimalConfigTest, GenericRetainsEveryEffectiveParameter)
{
  expectSameEffectiveParameters<Slam3dLocalizer>("slam_3d_localization.yaml");
}

TEST_F(MinimalConfigTest, GenericOmittedTopicsUseTheUnmodifiedCppDefaults)
{
  const auto defaults = snapshotFiles<Slam3dLocalizer>({});
  const auto configured = snapshot<Slam3dLocalizer>("slam_3d_localization.yaml", false);
  EXPECT_EQ(defaults.at("cloud_topic").as_string(), "/slam_3d/points/center");
  EXPECT_EQ(defaults.at("odom_topic").as_string(), "/odometry/filtered");
  for (const auto * key : {"cloud_topic", "odom_topic"}) {
    SCOPED_TRACE(key);
    EXPECT_EQ(
      configured.at(key).get_parameter_value(), defaults.at(key).get_parameter_value());
  }
}

TEST_F(MinimalConfigTest, AvsRetainsEveryEffectiveParameter)
{
  expectSameEffectiveParameters<Slam3dLocalizer>("slam_3d_localization_avs.yaml");
}

TEST_F(MinimalConfigTest, LioChangesOnlyPointTimeModeToAuto)
{
  EXPECT_EQ(
    snapshot<LioOdometry>("slam_3d_lio_gangnam.yaml", true).at("point_time_mode").as_string(),
    "absolute_seconds");
  expectSameEffectiveParameters<LioOdometry>(
    "slam_3d_lio_gangnam.yaml", {rclcpp::Parameter("point_time_mode", "auto")});
}

TEST_F(MinimalConfigTest, LioOmittedSensorValuesUseTheUnmodifiedCppDefaults)
{
  const auto defaults = snapshotFiles<LioOdometry>({});
  const auto configured = snapshot<LioOdometry>("slam_3d_lio_gangnam.yaml", false);
  EXPECT_EQ(defaults.at("cloud_topic").as_string(), "/hesai/center/lidar_points");
  EXPECT_EQ(defaults.at("imu_topic").as_string(), "/imu/data");
  EXPECT_EQ(defaults.at("imu_frame").as_string(), "cv7_link");
  EXPECT_EQ(defaults.at("point_time_field").as_string(), "timestamp");
  for (const auto * key : {"cloud_topic", "imu_topic", "imu_frame", "point_time_field"}) {
    SCOPED_TRACE(key);
    EXPECT_EQ(
      configured.at(key).get_parameter_value(), defaults.at(key).get_parameter_value());
  }
  EXPECT_EQ(defaults.at("point_time_mode").as_string(), "auto");
  EXPECT_EQ(configured.at("point_time_mode").as_string(), "auto");
  EXPECT_EQ(
    configured.at("imu_position_in_base_m").as_double_array(),
    (std::vector<double>{0.0, 0.0, 1.20984}));
}

TEST_F(MinimalConfigTest, LioMapLocalizerRetainsEveryEffectiveParameter)
{
  expectSameEffectiveParameters<Slam3dLocalizer>("slam_3d_lio_gangnam.yaml");
}

TEST_F(MinimalConfigTest, IndoorRetainsEveryEffectiveParameter)
{
  expectSameEffectiveParameters<Slam3dLocalizer>(
    "slam_3d_localization_indoor_xt32_no_wheel_replay.yaml");
}

TEST_F(MinimalConfigTest, UserFileOverridesPipelineDefaultsAndRuntimeWinsLast)
{
  const std::filesystem::path root(SLAM_3D_LOCALIZATION_SOURCE_DIR);
  const std::vector<std::string> files{
    SLAM_3D_LOCALIZATION_LIO_PROFILE_FILE,
    (root / "config" / "slam_3d_lio_gangnam.yaml").string(),
    (root / "test" / "fixtures" / "lio_user_overrides.yaml").string()};
  const std::vector<rclcpp::Parameter> runtime{
    rclcpp::Parameter("use_sim_time", true)};

  const auto lio = snapshotFiles<LioOdometry>(files, runtime);
  EXPECT_EQ(lio.at("odom_topic").as_string(), "/minimal_config/user_odom");
  EXPECT_EQ(lio.at("cloud_topic").as_string(), "/minimal_config/user_cloud");
  EXPECT_FALSE(lio.at("imu_enabled").as_bool());
  EXPECT_TRUE(lio.at("use_sim_time").as_bool());

  const auto localizer = snapshotFiles<Slam3dLocalizer>(files, runtime);
  EXPECT_EQ(localizer.at("odom_topic").as_string(), "/minimal_config/user_odom");
  EXPECT_EQ(localizer.at("cloud_subscription_depth").as_int(), 7);
  EXPECT_FALSE(localizer.at("require_exact_odom_for_update").as_bool());
  EXPECT_FALSE(localizer.at("publish_tf").as_bool());
  EXPECT_TRUE(localizer.at("use_sim_time").as_bool());
  EXPECT_EQ(localizer.at("map_path").as_string(), map_path_);
}

TEST_F(MinimalConfigTest, PendingRetryTimerSleepsUntilExactOdometryIsNeeded)
{
  auto node = timerTestNode();
  EXPECT_TRUE(Slam3dLocalizerTestPeer::timerCanceled(*node));
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*node), 0U);

  Slam3dLocalizerTestPeer::cloud(*node, 100);
  Slam3dLocalizerTestPeer::cloud(*node, 101);
  EXPECT_FALSE(Slam3dLocalizerTestPeer::timerCanceled(*node));
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*node), 2U);
  Slam3dLocalizerTestPeer::retry(*node);
  EXPECT_FALSE(Slam3dLocalizerTestPeer::timerCanceled(*node));
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*node), 2U);

  Slam3dLocalizerTestPeer::exactOdom(*node, 100);
  Slam3dLocalizerTestPeer::retry(*node);
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*node), 1U);
  EXPECT_FALSE(Slam3dLocalizerTestPeer::timerCanceled(*node));
  Slam3dLocalizerTestPeer::exactOdom(*node, 101);
  Slam3dLocalizerTestPeer::retry(*node);
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*node), 0U);
  EXPECT_TRUE(Slam3dLocalizerTestPeer::timerCanceled(*node));

  // A later unmatched scan re-arms demand without an always-running timer.
  Slam3dLocalizerTestPeer::cloud(*node, 102);
  EXPECT_FALSE(Slam3dLocalizerTestPeer::timerCanceled(*node));
}

TEST_F(MinimalConfigTest, ImmediateExactAndGenericScansLeaveRetryTimerCanceled)
{
  auto exact = timerTestNode();
  Slam3dLocalizerTestPeer::exactOdom(*exact, 100);
  Slam3dLocalizerTestPeer::cloud(*exact, 100);
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*exact), 0U);
  EXPECT_TRUE(Slam3dLocalizerTestPeer::timerCanceled(*exact));

  auto generic = timerTestNode(false);
  Slam3dLocalizerTestPeer::cloud(*generic, 100);
  Slam3dLocalizerTestPeer::retry(*generic);
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*generic), 0U);
  EXPECT_TRUE(Slam3dLocalizerTestPeer::timerCanceled(*generic));
}

TEST_F(MinimalConfigTest, InitialPoseCancelsPendingRetryOnlyWhenPoseIsAccepted)
{
  auto node = timerTestNode();
  Slam3dLocalizerTestPeer::cloud(*node, 100);
  Slam3dLocalizerTestPeer::initialPose(*node, "incorrect_frame");
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*node), 1U);
  EXPECT_FALSE(Slam3dLocalizerTestPeer::timerCanceled(*node));
  Slam3dLocalizerTestPeer::initialPose(*node);
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*node), 0U);
  EXPECT_TRUE(Slam3dLocalizerTestPeer::timerCanceled(*node));
  Slam3dLocalizerTestPeer::cloud(*node, 101);
  EXPECT_FALSE(Slam3dLocalizerTestPeer::timerCanceled(*node));
}

TEST_F(MinimalConfigTest, CloudEpochResetClearsQueueAndCancelsRetry)
{
  auto node = timerTestNode();
  Slam3dLocalizerTestPeer::cloud(*node, 100);
  Slam3dLocalizerTestPeer::cloud(*node, 101);
  Slam3dLocalizerTestPeer::exactOdom(*node, 100);
  Slam3dLocalizerTestPeer::seedLastCloudStamp(*node, 1000);
  Slam3dLocalizerTestPeer::retry(*node);
  EXPECT_EQ(Slam3dLocalizerTestPeer::pendingCount(*node), 0U);
  EXPECT_TRUE(Slam3dLocalizerTestPeer::timerCanceled(*node));
}

}  // namespace
}  // namespace gole::slam_3d_localization
