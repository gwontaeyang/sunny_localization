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

void expectFixedZeroImuBias(const LioStateEstimator & estimator)
{
  EXPECT_DOUBLE_EQ(estimator.gyroscopeBias().squaredNorm(), 0.0);
  EXPECT_DOUBLE_EQ(estimator.accelerometerBias().squaredNorm(), 0.0);
  EXPECT_DOUBLE_EQ(estimator.covariance().bottomRows<6>().squaredNorm(), 0.0);
  EXPECT_DOUBLE_EQ(estimator.covariance().rightCols<6>().squaredNorm(), 0.0);
  EXPECT_TRUE(estimator.covariance().allFinite());
  EXPECT_TRUE(estimator.covariance().isApprox(estimator.covariance().transpose(), 1.0e-12));
  EXPECT_GE(estimator.covariance().diagonal().minCoeff(), 0.0);
}

TEST(LioStateEstimator, DefaultIgnoresSuppliedBiasesAndKeepsTheirCovarianceZero)
{
  EXPECT_FALSE(LioStateEstimator::Config{}.imu_bias_estimation_enabled);
  LioStateEstimator estimator;
  expectFixedZeroImuBias(estimator);
  ASSERT_TRUE(
    estimator.initialize(
      stationaryMeasurement(kSecondNs), Eigen::Quaterniond::Identity(),
      Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0.4, -0.5, 0.6)));
  expectFixedZeroImuBias(estimator);
  EXPECT_GT(estimator.covariance()(0, 0), 0.0);
  EXPECT_GT(estimator.covariance()(3, 3), 0.0);
  EXPECT_GT(estimator.covariance()(6, 6), 0.0);

  estimator.reset();
  EXPECT_FALSE(estimator.initialized());
  expectFixedZeroImuBias(estimator);
}

TEST(LioStateEstimator, FixedBiasPropagationUsesUncorrectedGyroscopeAndAcceleration)
{
  LioStateEstimator estimator;
  ImuMeasurement measurement = stationaryMeasurement(2 * kSecondNs);
  measurement.angular_velocity_radps.z() = 0.2;
  measurement.linear_acceleration_mps2.z() += 1.0;
  ASSERT_TRUE(
    estimator.initialize(
      measurement, Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.0, 0.0, 0.2),
      Eigen::Vector3d(0.0, 0.0, 1.0)));

  for (int index = 1; index <= 100; ++index) {
    measurement.stamp_ns = 2 * kSecondNs + index * 10000000LL;
    ASSERT_EQ(estimator.propagate(measurement), ImuPropagationStatus::kApplied);
    expectFixedZeroImuBias(estimator);
  }

  EXPECT_NEAR(estimator.position().z(), 0.5, 1.0e-10);
  EXPECT_NEAR(estimator.velocity().z(), 1.0, 1.0e-10);
  const Eigen::Quaterniond expected_orientation(
    Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ()));
  EXPECT_NEAR(estimator.orientation().angularDistance(expected_orientation), 0.0, 1.0e-10);
}

TEST(LioStateEstimator, FixedBiasSurvivesRepeatedPoseWheelUpdatesAndGapRecovery)
{
  LioStateEstimator estimator;
  ImuMeasurement measurement = stationaryMeasurement(3 * kSecondNs);
  measurement.angular_velocity_radps.z() = 0.1;
  measurement.linear_acceleration_mps2.x() = 0.3;
  ASSERT_TRUE(estimator.initialize(measurement, Eigen::Quaterniond::Identity()));

  for (int index = 1; index <= 100; ++index) {
    measurement.stamp_ns = 3 * kSecondNs + index * 10000000LL;
    ASSERT_EQ(estimator.propagate(measurement), ImuPropagationStatus::kApplied);
    expectFixedZeroImuBias(estimator);
    if (index % 10 != 0) {
      continue;
    }

    Eigen::Isometry3d measured_pose = estimator.odomTBody();
    measured_pose.translation() += Eigen::Vector3d(0.05, -0.02, 0.01);
    const double position_error_before =
      (estimator.position() - measured_pose.translation()).norm();
    ASSERT_TRUE(estimator.correctBodyPose(measured_pose, 0.02, 0.02));
    EXPECT_LT((estimator.position() - measured_pose.translation()).norm(), position_error_before);
    expectFixedZeroImuBias(estimator);

    const double speed_before = (estimator.orientation().conjugate() * estimator.velocity()).x();
    const double measured_speed = speed_before + 0.1;
    ASSERT_TRUE(estimator.correctBodyForwardVelocity(measured_speed, 0.02));
    const double speed_after = (estimator.orientation().conjugate() * estimator.velocity()).x();
    EXPECT_LT(std::abs(speed_after - measured_speed), 0.1);
    expectFixedZeroImuBias(estimator);
  }

  const Eigen::Isometry3d pose_before_gap = estimator.odomTBody();
  measurement.stamp_ns += kSecondNs;
  ASSERT_EQ(estimator.propagate(measurement), ImuPropagationStatus::kGapTooLarge);
  expectFixedZeroImuBias(estimator);
  ASSERT_TRUE(estimator.resynchronizeAfterGap(measurement, 0.2, 0.5, 0.1));
  EXPECT_TRUE(estimator.odomTBody().matrix().isApprox(pose_before_gap.matrix(), 1.0e-12));
  expectFixedZeroImuBias(estimator);
  measurement.stamp_ns += 10000000LL;
  ASSERT_EQ(estimator.propagate(measurement), ImuPropagationStatus::kApplied);
  expectFixedZeroImuBias(estimator);
}

TEST(LioStateEstimator, OptionalBiasEstimationPreservesSuppliedBiasCompensation)
{
  LioStateEstimator::Config config;
  config.imu_bias_estimation_enabled = true;
  LioStateEstimator estimator(config);
  ImuMeasurement measurement = stationaryMeasurement(5 * kSecondNs);
  const Eigen::Vector3d gyro_bias(0.0, 0.0, 0.2);
  const Eigen::Vector3d accel_bias(0.4, -0.2, 0.1);
  measurement.angular_velocity_radps = gyro_bias;
  measurement.linear_acceleration_mps2 += accel_bias;
  ASSERT_TRUE(
    estimator.initialize(measurement, Eigen::Quaterniond::Identity(), gyro_bias, accel_bias));
  EXPECT_TRUE(estimator.gyroscopeBias().isApprox(gyro_bias, 1.0e-12));
  EXPECT_TRUE(estimator.accelerometerBias().isApprox(accel_bias, 1.0e-12));
  EXPECT_GT(estimator.covariance()(9, 9), 0.0);
  EXPECT_GT(estimator.covariance()(12, 12), 0.0);

  for (int index = 1; index <= 100; ++index) {
    measurement.stamp_ns = 5 * kSecondNs + index * 10000000LL;
    ASSERT_EQ(estimator.propagate(measurement), ImuPropagationStatus::kApplied);
  }
  EXPECT_NEAR(estimator.position().norm(), 0.0, 1.0e-10);
  EXPECT_NEAR(estimator.velocity().norm(), 0.0, 1.0e-10);
  EXPECT_NEAR(estimator.orientation().angularDistance(Eigen::Quaterniond::Identity()), 0.0, 1.0e-10);
}

TEST(LioStateEstimator, OptionalBiasEstimationRespondsToLidarPoseCorrection)
{
  LioStateEstimator::Config config;
  config.imu_bias_estimation_enabled = true;
  LioStateEstimator estimator(config);
  ImuMeasurement measurement = stationaryMeasurement(7 * kSecondNs);
  measurement.angular_velocity_radps.z() = 0.1;
  measurement.linear_acceleration_mps2.z() += 0.2;
  ASSERT_TRUE(estimator.initialize(measurement, Eigen::Quaterniond::Identity()));
  for (int index = 1; index <= 100; ++index) {
    measurement.stamp_ns = 7 * kSecondNs + index * 10000000LL;
    ASSERT_EQ(estimator.propagate(measurement), ImuPropagationStatus::kApplied);
  }

  ASSERT_TRUE(estimator.correctBodyPose(Eigen::Isometry3d::Identity(), 0.01, 0.01));
  EXPECT_GT(estimator.gyroscopeBias().z(), 1.0e-5);
  EXPECT_GT(estimator.accelerometerBias().z(), 1.0e-5);
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
