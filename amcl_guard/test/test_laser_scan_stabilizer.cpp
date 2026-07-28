#include "amcl_guard/laser_scan_stabilizer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{

constexpr double kPi = 3.14159265358979323846;

sensor_msgs::msg::LaserScan makeScan()
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "base_link";
  scan.angle_min = -static_cast<float>(kPi / 2.0);
  scan.angle_max = static_cast<float>(kPi / 2.0);
  scan.angle_increment = static_cast<float>(kPi / 4.0);
  scan.range_min = 0.05F;
  scan.range_max = 10.0F;
  scan.ranges.assign(5, std::numeric_limits<float>::infinity());
  return scan;
}

geometry_msgs::msg::TransformStamped makeTransform(
  const double x,
  const double y,
  const double yaw)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.rotation.z = std::sin(yaw / 2.0);
  transform.transform.rotation.w = std::cos(yaw / 2.0);
  return transform;
}

TEST(LaserScanStabilizer, IdentityPreservesValidBeam)
{
  auto source = makeScan();
  source.ranges[2] = 1.25F;
  const auto result = amcl_guard::warpLaserScan(
    source, makeScan(), makeTransform(0.0, 0.0, 0.0), 1);

  ASSERT_TRUE(result);
  EXPECT_EQ(1U, result->valid_beam_count);
  EXPECT_NEAR(1.25, result->scan.ranges[2], 1.0e-6);
  EXPECT_TRUE(result->scan.intensities.empty());
  EXPECT_FLOAT_EQ(0.0F, result->scan.time_increment);
}

TEST(LaserScanStabilizer, TranslationMovesWallTowardCurrentPose)
{
  auto source = makeScan();
  source.ranges[2] = 1.0F;
  const auto result = amcl_guard::warpLaserScan(
    source, makeScan(), makeTransform(-0.2, 0.0, 0.0), 1);

  ASSERT_TRUE(result);
  EXPECT_NEAR(0.8, result->scan.ranges[2], 1.0e-6);
}

TEST(LaserScanStabilizer, YawRotatesBeamIntoCurrentBin)
{
  auto source = makeScan();
  source.ranges[2] = 1.0F;
  const auto result = amcl_guard::warpLaserScan(
    source, makeScan(), makeTransform(0.0, 0.0, kPi / 2.0), 1);

  ASSERT_TRUE(result);
  EXPECT_NEAR(1.0, result->scan.ranges[4], 1.0e-6);
}

TEST(LaserScanStabilizer, RejectsInsufficientOutputBeams)
{
  auto source = makeScan();
  source.ranges[2] = 1.0F;
  EXPECT_FALSE(
    amcl_guard::warpLaserScan(
      source, makeScan(), makeTransform(0.0, 0.0, 0.0), 2));
}

TEST(LaserScanStabilizer, ReportsPlanarAndOutOfPlaneMotion)
{
  auto transform = makeTransform(0.3, 0.4, 0.2);
  const double roll = 0.04;
  const double pitch = -0.03;
  const double yaw = 0.2;
  const double cr = std::cos(roll / 2.0);
  const double sr = std::sin(roll / 2.0);
  const double cp = std::cos(pitch / 2.0);
  const double sp = std::sin(pitch / 2.0);
  const double cy = std::cos(yaw / 2.0);
  const double sy = std::sin(yaw / 2.0);
  transform.transform.rotation.x = sr * cp * cy - cr * sp * sy;
  transform.transform.rotation.y = cr * sp * cy + sr * cp * sy;
  transform.transform.rotation.z = cr * cp * sy - sr * sp * cy;
  transform.transform.rotation.w = cr * cp * cy + sr * sp * sy;

  const auto motion = amcl_guard::relativeMotion(transform);
  ASSERT_TRUE(motion.finite);
  EXPECT_NEAR(0.5, motion.translation_m, 1.0e-9);
  EXPECT_NEAR(yaw, motion.yaw_rad, 1.0e-9);
  EXPECT_NEAR(std::hypot(roll, pitch), motion.roll_pitch_rad, 1.0e-9);
}

}  // namespace
