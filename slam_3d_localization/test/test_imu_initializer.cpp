// Copyright 2026 Gole Robotics
#include "slam_3d_localization/imu_initializer.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace gole::slam_3d_localization
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TEST(StationaryImuInitializer, AlignsMeasuredSpecificForceWithOdomUp)
{
  ImuInitializationConfig config;
  config.min_samples = 10;
  config.min_duration_s = 0.09;
  StationaryImuInitializer initializer(config);
  const Eigen::Quaterniond expected_orientation(
    Eigen::AngleAxisd(12.0 * kPi / 180.0, Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d acceleration = expected_orientation.conjugate() *
    Eigen::Vector3d(0.0, 0.0, config.gravity_mps2);
  for (int index = 0; index < 10; ++index) {
    ImuMeasurement sample;
    sample.stamp_ns = 1000000000LL + index * 10000000LL;
    sample.angular_velocity_radps = Eigen::Vector3d(0.001, -0.002, 0.003);
    sample.linear_acceleration_mps2 = acceleration;
    ASSERT_TRUE(initializer.add(sample));
  }

  const auto result = initializer.evaluate();

  ASSERT_TRUE(result.evaluated);
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_NEAR(result.odom_q_base.angularDistance(expected_orientation), 0.0, 1.0e-9);
  EXPECT_NEAR(result.accelerometer_bias_mps2.norm(), 0.0, 1.0e-9);
  EXPECT_TRUE(
    result.gyroscope_bias_radps.isApprox(
      Eigen::Vector3d(0.001, -0.002, 0.003), 1.0e-12));
}

TEST(StationaryImuInitializer, RejectsMovingWindow)
{
  ImuInitializationConfig config;
  config.min_samples = 10;
  config.min_duration_s = 0.09;
  StationaryImuInitializer initializer(config);
  for (int index = 0; index < 10; ++index) {
    ImuMeasurement sample;
    sample.stamp_ns = 2000000000LL + index * 10000000LL;
    sample.angular_velocity_radps = Eigen::Vector3d(0.0, 0.0, 0.2);
    sample.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.80665);
    initializer.add(sample);
  }

  const auto result = initializer.evaluate();

  EXPECT_TRUE(result.evaluated);
  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("gyroscope_mean_too_large"), std::string::npos);
}

TEST(StationaryImuInitializer, TimestampRewindStartsANewWindow)
{
  ImuInitializationConfig config;
  config.min_samples = 3;
  config.min_duration_s = 0.02;
  StationaryImuInitializer initializer(config);
  ImuMeasurement sample;
  sample.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.80665);
  sample.stamp_ns = 100;
  initializer.add(sample);
  sample.stamp_ns = 200;
  initializer.add(sample);
  sample.stamp_ns = 50;
  initializer.add(sample);

  EXPECT_EQ(initializer.size(), 1U);
}

}  // namespace
}  // namespace gole::slam_3d_localization
