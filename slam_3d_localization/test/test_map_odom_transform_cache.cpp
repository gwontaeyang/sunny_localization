// Copyright 2026 Gole Robotics
#include "slam_3d_localization/map_odom_transform_cache.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>

namespace gole::slam_3d_localization
{
namespace
{

Eigen::Isometry3d makePose(
  const Eigen::Vector3d & translation, const Eigen::Vector3d & axis,
  double angle_rad)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = translation;
  pose.linear() = Eigen::AngleAxisd(angle_rad, axis.normalized()).toRotationMatrix();
  return pose;
}

TEST(MapOdomTransformCache, StartsInvalid)
{
  MapOdomTransformCache cache;
  EXPECT_FALSE(cache.value().has_value());
  EXPECT_FALSE(cache.takeForOdomStamp(100).has_value());
}

TEST(MapOdomTransformCache, IdentityBootstrapFollowsMonotonicOdomBeforeLocalization)
{
  MapOdomTransformCache cache;

  const auto first = cache.takeForOdomStamp(100, true);
  ASSERT_TRUE(first.has_value());
  EXPECT_TRUE(
    first->matrix().isApprox(Eigen::Isometry3d::Identity().matrix(), 1.0e-12));
  EXPECT_FALSE(cache.takeForOdomStamp(99, true).has_value());
  EXPECT_FALSE(cache.takeForOdomStamp(100, true).has_value());
  EXPECT_TRUE(cache.takeForOdomStamp(101, true).has_value());
}

TEST(MapOdomTransformCache, FirstLocalizationReplacesIdentityBootstrap)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(cache.takeForOdomStamp(100, true).has_value());
  const Eigen::Isometry3d map_T_base = makePose(
    Eigen::Vector3d(3.0, -2.0, 0.5), Eigen::Vector3d::UnitZ(), 0.2);
  ASSERT_TRUE(cache.reanchor(map_T_base, Eigen::Isometry3d::Identity(), 101));

  EXPECT_FALSE(cache.takeForOdomStamp(101, true).has_value());
  const auto corrected = cache.takeForOdomStamp(102, true);
  ASSERT_TRUE(corrected.has_value());
  EXPECT_TRUE(corrected->matrix().isApprox(map_T_base.matrix(), 1.0e-12));
}

TEST(MapOdomTransformCache, ExplicitRetirementStopsIdentityBootstrapPermanently)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(cache.takeForOdomStamp(100, true).has_value());

  cache.retireIdentityBootstrap();
  cache.clearCorrectionPreserveWatermark();

  EXPECT_FALSE(cache.takeForOdomStamp(101, true).has_value());
  cache.resetOdomEpoch();
  EXPECT_FALSE(cache.takeForOdomStamp(1, true).has_value());
}

TEST(MapOdomTransformCache, OdomEpochResetRetiresStartupIdentityAssumption)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(cache.takeForOdomStamp(100, true).has_value());

  cache.resetOdomEpoch();

  EXPECT_FALSE(cache.takeForOdomStamp(1, true).has_value());
}

TEST(MapOdomTransformCache, NeverFallsBackToIdentityAfterLocalization)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 10));
  ASSERT_TRUE(cache.takeForOdomStamp(20, true).has_value());

  cache.clearCorrectionPreserveWatermark();
  EXPECT_FALSE(cache.takeForOdomStamp(21, true).has_value());
  cache.resetOdomEpoch();
  EXPECT_FALSE(cache.takeForOdomStamp(1, true).has_value());
}

TEST(MapOdomTransformCache, ReanchorsWithMapBaseTimesInverseOdomBase)
{
  const Eigen::Isometry3d map_T_base = makePose(
    Eigen::Vector3d(7.0, -2.0, 1.5), Eigen::Vector3d(0.2, 0.7, -0.4), 0.8);
  const Eigen::Isometry3d odom_T_base = makePose(
    Eigen::Vector3d(-3.0, 4.0, 0.4), Eigen::Vector3d(-0.5, 0.1, 0.8), -0.35);
  MapOdomTransformCache cache;

  ASSERT_TRUE(cache.reanchor(map_T_base, odom_T_base, 10));
  const auto map_T_odom = cache.value();
  ASSERT_TRUE(map_T_odom.has_value());
  EXPECT_TRUE(map_T_odom->matrix().isApprox(
      (map_T_base * odom_T_base.inverse()).matrix(), 1.0e-12));
  EXPECT_TRUE(((*map_T_odom) * odom_T_base).matrix().isApprox(
      map_T_base.matrix(), 1.0e-12));
}

TEST(MapOdomTransformCache, ReanchorReplacesPreviousAcceptedLocalization)
{
  MapOdomTransformCache cache;
  const Eigen::Isometry3d first_map_T_base = makePose(
    Eigen::Vector3d(1.0, 2.0, 0.0), Eigen::Vector3d::UnitZ(), 0.1);
  const Eigen::Isometry3d first_odom_T_base = makePose(
    Eigen::Vector3d(0.5, 0.2, 0.0), Eigen::Vector3d::UnitZ(), -0.05);
  ASSERT_TRUE(cache.reanchor(first_map_T_base, first_odom_T_base, 10));
  const Eigen::Isometry3d first = *cache.value();

  const Eigen::Isometry3d second_map_T_base = makePose(
    Eigen::Vector3d(4.0, -1.0, 0.3), Eigen::Vector3d::UnitY(), 0.2);
  const Eigen::Isometry3d second_odom_T_base = makePose(
    Eigen::Vector3d(2.0, 3.0, -0.2), Eigen::Vector3d::UnitX(), -0.1);
  ASSERT_TRUE(cache.reanchor(second_map_T_base, second_odom_T_base, 20));
  const auto second = cache.value();

  ASSERT_TRUE(second.has_value());
  EXPECT_FALSE(second->matrix().isApprox(first.matrix(), 1.0e-12));
  EXPECT_TRUE(((*second) * second_odom_T_base).matrix().isApprox(
      second_map_T_base.matrix(), 1.0e-12));
}

TEST(MapOdomTransformCache, CorrectionClearSuppressesTheCachedTransform)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 10));
  ASSERT_TRUE(cache.value().has_value());

  cache.clearCorrectionPreserveWatermark();

  EXPECT_FALSE(cache.value().has_value());
}

TEST(MapOdomTransformCache, NonfiniteReanchorClearsCorrectionButPreservesWatermark)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 10));
  ASSERT_TRUE(cache.takeForOdomStamp(20).has_value());
  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(cache.reanchor(invalid, Eigen::Isometry3d::Identity(), 30));
  EXPECT_FALSE(cache.value().has_value());
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 5));
  EXPECT_FALSE(cache.takeForOdomStamp(20).has_value());
  EXPECT_TRUE(cache.takeForOdomStamp(21).has_value());
}

TEST(MapOdomTransformCache, EmitsOnlyAtStrictlyIncreasingOdomStamps)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 100));

  EXPECT_FALSE(cache.takeForOdomStamp(99).has_value());
  EXPECT_FALSE(cache.takeForOdomStamp(100).has_value());
  EXPECT_TRUE(cache.takeForOdomStamp(101).has_value());
  EXPECT_FALSE(cache.takeForOdomStamp(101).has_value());
  EXPECT_TRUE(cache.takeForOdomStamp(102).has_value());
}

TEST(MapOdomTransformCache, ReanchorDoesNotReopenStampGateForQueuedOdom)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 90));
  ASSERT_TRUE(cache.takeForOdomStamp(100).has_value());
  const Eigen::Isometry3d updated_map_T_base = makePose(
    Eigen::Vector3d(3.0, -2.0, 0.5), Eigen::Vector3d::UnitZ(), 0.2);

  ASSERT_TRUE(
    cache.reanchor(updated_map_T_base, Eigen::Isometry3d::Identity(), 95));

  EXPECT_FALSE(cache.takeForOdomStamp(99).has_value());
  EXPECT_FALSE(cache.takeForOdomStamp(100).has_value());
  const auto emitted = cache.takeForOdomStamp(101);
  ASSERT_TRUE(emitted.has_value());
  EXPECT_TRUE(emitted->matrix().isApprox(updated_map_T_base.matrix(), 1.0e-12));
}

TEST(MapOdomTransformCache, OdomEpochResetClearsCorrectionAndStampGate)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 90));
  ASSERT_TRUE(cache.takeForOdomStamp(100).has_value());

  cache.resetOdomEpoch();
  EXPECT_FALSE(cache.takeForOdomStamp(10).has_value());
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 9));

  EXPECT_TRUE(cache.takeForOdomStamp(10).has_value());
}

TEST(MapOdomTransformCache, CorrectionClearPreservesLastEmittedWatermark)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 90));
  ASSERT_TRUE(cache.takeForOdomStamp(100).has_value());

  cache.clearCorrectionPreserveWatermark();
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 50));

  EXPECT_FALSE(cache.takeForOdomStamp(99).has_value());
  EXPECT_FALSE(cache.takeForOdomStamp(100).has_value());
  EXPECT_TRUE(cache.takeForOdomStamp(101).has_value());
}

TEST(MapOdomTransformCache, CloudEpochClearSuppressesOdomUntilFreshReanchor)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 90));
  ASSERT_TRUE(cache.takeForOdomStamp(100).has_value());

  cache.clearCorrectionPreserveWatermark();

  EXPECT_FALSE(cache.takeForOdomStamp(101).has_value());
  EXPECT_FALSE(cache.takeForOdomStamp(1000).has_value());
}

TEST(MapOdomTransformCache, AnchorAheadOfWatermarkSuppressesQueuedOdom)
{
  MapOdomTransformCache cache;
  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 90));
  ASSERT_TRUE(cache.takeForOdomStamp(100).has_value());

  ASSERT_TRUE(
    cache.reanchor(Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity(), 110));

  EXPECT_FALSE(cache.takeForOdomStamp(101).has_value());
  EXPECT_FALSE(cache.takeForOdomStamp(110).has_value());
  EXPECT_TRUE(cache.takeForOdomStamp(111).has_value());
}

}  // namespace
}  // namespace gole::slam_3d_localization
