// Copyright 2026 Gole Robotics
#include "slam_3d_localization/cloud_freshness_guard.hpp"
#include "slam_3d_localization/map_odom_transform_cache.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace gole::slam_3d_localization
{
namespace
{

constexpr std::int64_t kSecondNs = 1000000000LL;

TEST(CloudFreshnessGuard, DisabledDefaultNeverDrops)
{
  const CloudFreshnessGuard guard(0.0);
  const auto result = guard.evaluate(100 * kSecondNs, 1 * kSecondNs);

  EXPECT_FALSE(result.enabled);
  EXPECT_TRUE(result.ros_time_started);
  EXPECT_FALSE(result.stale);
  EXPECT_DOUBLE_EQ(result.age_s, 99.0);
}

TEST(CloudFreshnessGuard, WaitsUntilRosTimeHasStarted)
{
  const CloudFreshnessGuard guard(0.5);
  const auto result = guard.evaluate(0, -10 * kSecondNs);

  EXPECT_TRUE(result.enabled);
  EXPECT_FALSE(result.ros_time_started);
  EXPECT_FALSE(result.stale);
}

TEST(CloudFreshnessGuard, DropsOnlyWhenAgeStrictlyExceedsLimit)
{
  const CloudFreshnessGuard guard(0.5);

  EXPECT_FALSE(guard.evaluate(10 * kSecondNs, 10 * kSecondNs).stale);
  EXPECT_FALSE(guard.evaluate(10 * kSecondNs, 9500000000LL).stale);
  EXPECT_TRUE(guard.evaluate(10 * kSecondNs + 1, 9500000000LL).stale);
}

TEST(CloudFreshnessGuard, FutureStampedCloudIsNotStale)
{
  const CloudFreshnessGuard guard(0.1);
  const auto result = guard.evaluate(10 * kSecondNs, 11 * kSecondNs);

  EXPECT_LT(result.age_s, 0.0);
  EXPECT_FALSE(result.stale);
}

TEST(CloudFreshnessGuard, RejectsInvalidConfiguration)
{
  EXPECT_THROW(CloudFreshnessGuard(-0.1), std::invalid_argument);
  EXPECT_THROW(
    CloudFreshnessGuard(std::numeric_limits<double>::infinity()),
    std::invalid_argument);
}

TEST(CloudFreshnessGuard, StaleRewoundEpochCannotKeepOldTfCorrectionAlive)
{
  MapOdomTransformCache tf_cache;
  ASSERT_TRUE(
    tf_cache.reanchor(
      Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(),
      100 * kSecondNs));
  ASSERT_TRUE(tf_cache.takeForOdomStamp(101 * kSecondNs).has_value());
  const CloudFreshnessGuard freshness_guard(1.0);

  // ROS time keeps moving forward while the cloud alone rewinds to a new
  // epoch. The callback's epoch preflight clears TF before this stale result
  // can return early.
  const auto rewound_cloud = freshness_guard.evaluate(
    200 * kSecondNs, 10 * kSecondNs);
  ASSERT_TRUE(rewound_cloud.stale);
  tf_cache.clearCorrectionPreserveWatermark();

  EXPECT_FALSE(tf_cache.takeForOdomStamp(201 * kSecondNs).has_value());
}

}  // namespace
}  // namespace gole::slam_3d_localization
