// Copyright 2026 Gole Robotics
#include "slam_3d_localization/odom_pose_buffer.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace gole::slam_3d_localization
{
namespace
{

constexpr std::int64_t kSecondNs = 1000000000LL;

Eigen::Isometry3d pose(double x, double yaw_rad = 0.0)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation().x() = x;
  result.linear() =
    Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

TEST(OdomPoseBuffer, InterpolatesTranslationAndRotationAtCloudStamp)
{
  OdomPoseBuffer buffer(5.0, 0.20, 0.05, 1.0);
  buffer.insert(1000 * kSecondNs, pose(0.0, 0.0));
  buffer.insert(1000 * kSecondNs + 100000000LL, pose(1.0, M_PI / 2.0));

  const auto result = buffer.lookup(1000 * kSecondNs + 50000000LL);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mode, OdomLookupMode::kInterpolated);
  EXPECT_NEAR(result.odom_T_base.translation().x(), 0.5, 1.0e-9);
  EXPECT_NEAR(
    Eigen::AngleAxisd(result.odom_T_base.linear()).angle(), M_PI / 4.0,
    1.0e-9);
  EXPECT_NEAR(result.bracket_gap_s, 0.10, 1.0e-12);
  EXPECT_NEAR(result.nearest_age_s, 0.05, 1.0e-12);
}

TEST(OdomPoseBuffer, RejectsCloudInLargeOdometryGap)
{
  OdomPoseBuffer buffer(5.0, 0.20, 0.05, 1.0);
  buffer.insert(1000 * kSecondNs, pose(0.0));
  buffer.insert(1001 * kSecondNs, pose(1.0));

  const auto result = buffer.lookup(1000 * kSecondNs + 500000000LL);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.mode, OdomLookupMode::kNearestPast);
  EXPECT_NEAR(result.bracket_gap_s, 1.0, 1.0e-12);
  EXPECT_NEAR(result.nearest_age_s, 0.5, 1.0e-12);
}

TEST(OdomPoseBuffer, AllowsBoundedNearestSampleAndRejectsStaleSample)
{
  OdomPoseBuffer buffer(5.0, 0.20, 0.05, 1.0);
  buffer.insert(1000 * kSecondNs, pose(3.0));

  const auto near = buffer.lookup(1000 * kSecondNs + 40000000LL);
  const auto stale = buffer.lookup(1000 * kSecondNs + 60000000LL);

  ASSERT_TRUE(near.valid);
  EXPECT_EQ(near.mode, OdomLookupMode::kNearestPast);
  EXPECT_NEAR(near.odom_T_base.translation().x(), 3.0, 1.0e-9);
  EXPECT_FALSE(stale.valid);
  EXPECT_EQ(stale.mode, OdomLookupMode::kNearestPast);
}

TEST(OdomPoseBuffer, ReplacesDuplicateAndPrunesByNewestStamp)
{
  OdomPoseBuffer buffer(1.0, 0.20, 0.05, 1.0);
  buffer.insert(1000 * kSecondNs, pose(1.0));
  buffer.insert(1000 * kSecondNs, pose(2.0));
  buffer.insert(1002 * kSecondNs, pose(4.0));

  EXPECT_EQ(buffer.size(), 1U);
  const auto result = buffer.lookup(1002 * kSecondNs);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.mode, OdomLookupMode::kExact);
  EXPECT_NEAR(result.odom_T_base.translation().x(), 4.0, 1.0e-9);
}

TEST(OdomPoseBuffer, RejectsFutureOnlyHoldForCausalOperation)
{
  OdomPoseBuffer buffer(5.0, 0.20, 0.05, 1.0);
  buffer.insert(1000 * kSecondNs + 40000000LL, pose(3.0));

  const auto result = buffer.lookup(1000 * kSecondNs);

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.mode, OdomLookupMode::kNearestFuture);
  EXPECT_NEAR(result.nearest_age_s, 0.04, 1.0e-12);
}

TEST(OdomPoseBuffer, ClearsBufferOnLargeTimestampRewind)
{
  OdomPoseBuffer buffer(5.0, 0.20, 0.05, 1.0);
  EXPECT_FALSE(buffer.insert(100 * kSecondNs, pose(1.0)));
  EXPECT_TRUE(buffer.insert(10 * kSecondNs, pose(2.0)));

  EXPECT_EQ(buffer.size(), 1U);
  const auto result = buffer.lookup(10 * kSecondNs);
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.odom_T_base.translation().x(), 2.0, 1.0e-9);
}

TEST(OdomPoseBuffer, ExplicitClearRemovesPriorEpochSamples)
{
  OdomPoseBuffer buffer(5.0, 0.20, 0.05, 1.0);
  buffer.insert(100 * kSecondNs, pose(1.0));

  buffer.clear();

  EXPECT_EQ(buffer.size(), 0U);
  EXPECT_FALSE(buffer.lookup(100 * kSecondNs).valid);
}

TEST(OdomPoseBuffer, RejectsNonFiniteTimingParameters)
{
  EXPECT_THROW(
    OdomPoseBuffer(
      std::numeric_limits<double>::quiet_NaN(), 0.20, 0.05, 1.0),
    std::invalid_argument);
}

}  // namespace
}  // namespace gole::slam_3d_localization
