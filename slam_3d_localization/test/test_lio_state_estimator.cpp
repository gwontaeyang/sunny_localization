// Copyright 2026 Gole Robotics
#include "slam_3d_localization/lio_state_estimator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace gole::slam_3d_localization
{
namespace
{

constexpr std::int64_t kSecondNs = 1000000000LL;
constexpr double kPi = 3.14159265358979323846;

ImuMeasurement stationaryMeasurement(const std::int64_t stamp_ns)
{
  ImuMeasurement measurement;
  measurement.stamp_ns = stamp_ns;
  measurement.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.80665);
  return measurement;
}

TEST(LioStateEstimator, StationarySpecificForceDoesNotCreateMotion)
{
  LioStateEstimator estimator;
  ASSERT_TRUE(
    estimator.initialize(
      stationaryMeasurement(10 * kSecondNs), Eigen::Quaterniond::Identity()));

  for (int index = 1; index <= 100; ++index) {
    EXPECT_EQ(
      estimator.propagate(stationaryMeasurement(10 * kSecondNs + index * 10000000LL)),
      ImuPropagationStatus::kApplied);
  }

  EXPECT_NEAR(estimator.position().norm(), 0.0, 1.0e-9);
  EXPECT_NEAR(estimator.velocity().norm(), 0.0, 1.0e-9);
  EXPECT_NEAR(estimator.orientation().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1.0e-9);
}

TEST(LioStateEstimator, PreservesSixDofRampOrientationAndVerticalMotion)
{
  LioStateEstimator estimator;
  const Eigen::Quaterniond ramp_orientation(
    Eigen::AngleAxisd(10.0 * kPi / 180.0, Eigen::Vector3d::UnitY()));
  ImuMeasurement measurement = stationaryMeasurement(20 * kSecondNs);
  measurement.linear_acceleration_mps2 =
    ramp_orientation.conjugate() * Eigen::Vector3d(0.0, 0.0, 9.80665);
  ASSERT_TRUE(estimator.initialize(measurement, ramp_orientation));

  for (int index = 1; index <= 100; ++index) {
    measurement.stamp_ns = 20 * kSecondNs + index * 10000000LL;
    ASSERT_EQ(estimator.propagate(measurement), ImuPropagationStatus::kApplied);
  }

  EXPECT_NEAR(estimator.position().z(), 0.0, 1.0e-8);
  EXPECT_NEAR(estimator.orientation().angularDistance(ramp_orientation), 0.0, 1.0e-9);

  Eigen::Isometry3d lidar_pose = Eigen::Isometry3d::Identity();
  lidar_pose.translation() = Eigen::Vector3d(1.0, 0.0, std::tan(10.0 * kPi / 180.0));
  lidar_pose.linear() = ramp_orientation.toRotationMatrix();
  ASSERT_TRUE(estimator.correctBodyPose(lidar_pose, 0.01, 0.01));
  EXPECT_GT(estimator.position().z(), 0.0);
  EXPECT_NEAR(
    Eigen::Quaterniond(estimator.odomTBody().linear()).angularDistance(ramp_orientation),
    0.0, 0.01);
}

TEST(LioStateEstimator, LidarPoseCorrectionReducesPoseError)
{
  LioStateEstimator estimator;
  ASSERT_TRUE(
    estimator.initialize(
      stationaryMeasurement(30 * kSecondNs), Eigen::Quaterniond::Identity()));
  Eigen::Isometry3d measured = Eigen::Isometry3d::Identity();
  measured.translation() = Eigen::Vector3d(2.0, -1.0, 0.5);
  measured.linear() = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const double before = (estimator.position() - measured.translation()).norm();
  ASSERT_TRUE(estimator.correctBodyPose(measured, 0.01, 0.01));
  const double after = (estimator.position() - measured.translation()).norm();

  EXPECT_LT(after, before);
  EXPECT_TRUE(estimator.odomTBody().matrix().allFinite());
  EXPECT_LT(estimator.orientation().angularDistance(Eigen::Quaterniond(measured.linear())), 0.2);
}

TEST(LioStateEstimator, LidarCovarianceDownweightsWeakTranslationAxis)
{
  LioStateEstimator estimator;
  ASSERT_TRUE(
    estimator.initialize(
      stationaryMeasurement(35 * kSecondNs), Eigen::Quaterniond::Identity()));
  Eigen::Isometry3d measured = Eigen::Isometry3d::Identity();
  measured.translation() = Eigen::Vector3d(1.0, 0.0, 1.0);
  Eigen::Matrix<double, 6, 6> measurement_covariance =
    Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  measurement_covariance(2, 2) = 100.0;

  ASSERT_TRUE(estimator.correctBodyPose(measured, measurement_covariance));

  EXPECT_GT(estimator.position().x(), 10.0 * estimator.position().z());
}

TEST(LioStateEstimator, RejectsBadTimeAndNonfiniteMeasurementsWithoutMutation)
{
  LioStateEstimator estimator;
  ASSERT_TRUE(
    estimator.initialize(
      stationaryMeasurement(40 * kSecondNs), Eigen::Quaterniond::Identity()));
  const Eigen::Isometry3d before = estimator.odomTBody();

  EXPECT_EQ(
    estimator.propagate(stationaryMeasurement(40 * kSecondNs)),
    ImuPropagationStatus::kNonmonotonicStamp);
  ImuMeasurement gap = stationaryMeasurement(41 * kSecondNs);
  EXPECT_EQ(estimator.propagate(gap), ImuPropagationStatus::kGapTooLarge);
  ImuMeasurement invalid = stationaryMeasurement(40 * kSecondNs + 10000000LL);
  invalid.angular_velocity_radps.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(estimator.propagate(invalid), ImuPropagationStatus::kInvalidMeasurement);

  EXPECT_TRUE(estimator.odomTBody().matrix().isApprox(before.matrix(), 1.0e-12));
  EXPECT_EQ(estimator.stampNs(), 40 * kSecondNs);
}

TEST(LioStateEstimator, GapResynchronizationPreservesPoseAndInflatesUncertainty)
{
  LioStateEstimator estimator;
  ASSERT_TRUE(
    estimator.initialize(
      stationaryMeasurement(45 * kSecondNs), Eigen::Quaterniond::Identity()));
  const Eigen::Isometry3d pose_before = estimator.odomTBody();
  const auto covariance_before = estimator.covariance();
  const ImuMeasurement after_gap = stationaryMeasurement(46 * kSecondNs);

  ASSERT_TRUE(estimator.resynchronizeAfterGap(after_gap, 0.2, 0.5, 0.1));

  EXPECT_EQ(estimator.stampNs(), after_gap.stamp_ns);
  EXPECT_TRUE(estimator.odomTBody().matrix().isApprox(pose_before.matrix(), 1.0e-12));
  EXPECT_GT(estimator.covariance()(0, 0), covariance_before(0, 0));
  EXPECT_GT(estimator.covariance()(3, 3), covariance_before(3, 3));
  EXPECT_GT(estimator.covariance()(6, 6), covariance_before(6, 6));
}

TEST(LioStateEstimator, OptionalWheelObservationIsDisabledByTheCallerButUsableLater)
{
  LioStateEstimator estimator;
  ASSERT_TRUE(
    estimator.initialize(
      stationaryMeasurement(50 * kSecondNs), Eigen::Quaterniond::Identity()));
  ImuMeasurement acceleration = stationaryMeasurement(50 * kSecondNs + 10000000LL);
  acceleration.linear_acceleration_mps2.x() = 2.0;
  ASSERT_EQ(estimator.propagate(acceleration), ImuPropagationStatus::kApplied);
  const double error_before = std::abs(estimator.velocity().x() - 0.5);

  ASSERT_TRUE(estimator.correctBodyForwardVelocity(0.5, 0.02));

  EXPECT_LT(std::abs(estimator.velocity().x() - 0.5), error_before);
}

}  // namespace
}  // namespace gole::slam_3d_localization
