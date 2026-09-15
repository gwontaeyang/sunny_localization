// Copyright 2026 Gole Robotics
#include "slam_3d_localization/lio_odometry.hpp"

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_field.hpp>

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace gole::slam_3d_localization
{

class LioOdometryTestPeer
{
public:
  static bool bounds(
    const sensor_msgs::msg::PointCloud2 & message, const std::string & field_name,
    const std::string & mode, const bool require_point_time,
    std::int64_t & start_stamp_ns, std::int64_t & end_stamp_ns, bool & has_point_time)
  {
    return LioOdometry::pointCloudTimeBounds(
      message, field_name, mode, 0.2, 0.02, require_point_time,
      start_stamp_ns, end_stamp_ns, has_point_time);
  }
};

namespace
{

template<typename T>
sensor_msgs::msg::PointCloud2 timedCloud(
  const std::vector<T> & values, const std::uint8_t datatype)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.stamp.sec = 100;
  message.height = 1U;
  message.width = static_cast<std::uint32_t>(values.size());
  message.is_bigendian = false;
  message.is_dense = true;
  message.point_step = sizeof(T);
  message.row_step = message.point_step * message.width;
  sensor_msgs::msg::PointField field;
  field.name = "time";
  field.offset = 0U;
  field.datatype = datatype;
  field.count = 1U;
  message.fields.push_back(field);
  message.data.resize(message.row_step);
  std::memcpy(message.data.data(), values.data(), message.data.size());
  return message;
}

TEST(LioPointTime, AcceptsRawAbsoluteFloat64Seconds)
{
  const auto message = timedCloud<double>(
    {100.0, 100.05, 100.1}, sensor_msgs::msg::PointField::FLOAT64);
  std::int64_t start = 0;
  std::int64_t end = 0;
  bool has_time = false;

  ASSERT_TRUE(
    LioOdometryTestPeer::bounds(
      message, "time", "absolute_seconds", true, start, end, has_time));
  EXPECT_TRUE(has_time);
  EXPECT_EQ(start, 100000000000LL);
  EXPECT_EQ(end, 100100000000LL);
}

TEST(LioPointTime, AutoMatchesExplicitAbsoluteModeForEpochSeconds)
{
  constexpr std::int32_t epoch_seconds = 1700000000;
  auto message = timedCloud<double>(
    {1700000000.0, 1700000000.05, 1700000000.1}, sensor_msgs::msg::PointField::FLOAT64);
  message.header.stamp.sec = epoch_seconds;
  std::int64_t auto_start = 0, auto_end = 0, explicit_start = 0, explicit_end = 0;
  bool auto_has_time = false, explicit_has_time = false;
  ASSERT_TRUE(LioOdometryTestPeer::bounds(
    message, "time", "auto", true, auto_start, auto_end, auto_has_time));
  ASSERT_TRUE(LioOdometryTestPeer::bounds(
    message, "time", "absolute_seconds", true,
    explicit_start, explicit_end, explicit_has_time));
  EXPECT_TRUE(auto_has_time);
  EXPECT_TRUE(explicit_has_time);
  EXPECT_EQ(auto_start, explicit_start);
  EXPECT_EQ(auto_end, explicit_end);
  EXPECT_EQ(auto_start, 1700000000000000000LL);
  EXPECT_GT(auto_end, auto_start);
}

TEST(LioPointTime, AutoMatchesExplicitRelativeSecondsMode)
{
  const auto message = timedCloud<double>(
    {0.0, 0.05, 0.1}, sensor_msgs::msg::PointField::FLOAT64);
  std::int64_t auto_start = 0, auto_end = 0, explicit_start = 0, explicit_end = 0;
  bool auto_has_time = false, explicit_has_time = false;
  ASSERT_TRUE(LioOdometryTestPeer::bounds(
    message, "time", "auto", true, auto_start, auto_end, auto_has_time));
  ASSERT_TRUE(LioOdometryTestPeer::bounds(
    message, "time", "relative_seconds", true,
    explicit_start, explicit_end, explicit_has_time));
  EXPECT_TRUE(auto_has_time);
  EXPECT_TRUE(explicit_has_time);
  EXPECT_EQ(auto_start, explicit_start);
  EXPECT_EQ(auto_end, explicit_end);
  EXPECT_EQ(auto_start, 100000000000LL);
}

TEST(LioPointTime, AutoDoesNotSilentlyAcceptSmallAbsoluteClockSeconds)
{
  // Auto treats these as offsets, not absolute times; a low-epoch sensor must
  // explicitly select absolute_seconds to retain the previous interpretation.
  const auto message = timedCloud<double>(
    {100.0, 100.05, 100.1}, sensor_msgs::msg::PointField::FLOAT64);
  std::int64_t start = 0, end = 0;
  bool has_time = false;
  EXPECT_FALSE(LioOdometryTestPeer::bounds(
    message, "time", "auto", true, start, end, has_time));
  EXPECT_TRUE(LioOdometryTestPeer::bounds(
    message, "time", "absolute_seconds", true, start, end, has_time));
}

TEST(LioPointTime, AcceptsCanonicalUint32NanosecondOffsets)
{
  const auto message = timedCloud<std::uint32_t>(
    {0U, 50000000U, 100000000U}, sensor_msgs::msg::PointField::UINT32);
  std::int64_t start = 0;
  std::int64_t end = 0;
  bool has_time = false;

  ASSERT_TRUE(
    LioOdometryTestPeer::bounds(
      message, "time", "relative_nanoseconds", true, start, end, has_time));
  EXPECT_TRUE(has_time);
  EXPECT_EQ(start, 100000000000LL);
  EXPECT_EQ(end, 100100000000LL);
}

TEST(LioPointTime, RequiredFieldAndHeaderEpochAreFailClosed)
{
  auto message = timedCloud<double>(
    {101.0, 101.05}, sensor_msgs::msg::PointField::FLOAT64);
  std::int64_t start = 0;
  std::int64_t end = 0;
  bool has_time = false;
  EXPECT_FALSE(
    LioOdometryTestPeer::bounds(
      message, "time", "absolute_seconds", true, start, end, has_time));

  message.fields.clear();
  EXPECT_FALSE(
    LioOdometryTestPeer::bounds(
      message, "time", "absolute_seconds", true, start, end, has_time));
  EXPECT_TRUE(
    LioOdometryTestPeer::bounds(
      message, "time", "absolute_seconds", false, start, end, has_time));
  EXPECT_FALSE(has_time);
}

TEST(LioPointTime, CanonicalModeRejectsWrongDatatype)
{
  const auto message = timedCloud<float>(
    {0.0F, 0.05F}, sensor_msgs::msg::PointField::FLOAT32);
  std::int64_t start = 0;
  std::int64_t end = 0;
  bool has_time = false;

  EXPECT_FALSE(
    LioOdometryTestPeer::bounds(
      message, "time", "relative_nanoseconds", true, start, end, has_time));
}

TEST(LioPointTime, BoundsUseExactlyTheRetainedDeskewSampleIndices)
{
  std::vector<double> timestamps(2048U, 100.05);
  // Stride two discards odd indices, including this unrelated timestamp.
  timestamps[1] = 999.0;
  timestamps[0] = 100.0;
  timestamps[2046] = 100.1;
  const auto message = timedCloud<double>(
    timestamps, sensor_msgs::msg::PointField::FLOAT64);
  std::int64_t start = 0;
  std::int64_t end = 0;
  bool has_time = false;
  ASSERT_TRUE(LioOdometryTestPeer::bounds(
    message, "time", "absolute_seconds", true, start, end, has_time));
  EXPECT_TRUE(has_time);
  EXPECT_EQ(start, 100000000000LL);
  EXPECT_EQ(end, 100100000000LL);
}

TEST(LioPointTime, InvalidRetainedTimestampStillRejectsTheScan)
{
  std::vector<double> timestamps(2048U, 100.05);
  timestamps[2] = 999.0;
  const auto message = timedCloud<double>(
    timestamps, sensor_msgs::msg::PointField::FLOAT64);
  std::int64_t start = 0;
  std::int64_t end = 0;
  bool has_time = false;
  EXPECT_FALSE(LioOdometryTestPeer::bounds(
    message, "time", "absolute_seconds", true, start, end, has_time));
}

}  // namespace
}  // namespace gole::slam_3d_localization
