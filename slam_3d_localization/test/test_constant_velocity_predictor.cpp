// Copyright 2026 Gole Robotics
#include "slam_3d_localization/constant_velocity_predictor.hpp"

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
constexpr double kPi = 3.14159265358979323846;

Eigen::Isometry3d pose(const double x, const double yaw_deg = 0.0)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation().x() = x;
  result.linear() = Eigen::AngleAxisd(
    yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}

Eigen::Isometry3d poseXyzRpy(
  const Eigen::Vector3d & translation, const Eigen::Vector3d & rpy_deg)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = translation;
  const Eigen::Quaterniond rotation =
    Eigen::AngleAxisd(rpy_deg.z() * kPi / 180.0, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy_deg.y() * kPi / 180.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy_deg.x() * kPi / 180.0, Eigen::Vector3d::UnitX());
  result.linear() = rotation.toRotationMatrix();
  return result;
}

double rotationAngleDeg(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle() * 180.0 / kPi;
}

TEST(ConstantVelocityPredictor, RequiresTwoMonotonicAcceptedPoses)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);

  auto result = predictor.predict(pose(0.0), 100 * kSecondNs);
  EXPECT_EQ(result.status, ConstantVelocityPredictionStatus::kNotReady);
  EXPECT_FALSE(result.applied);

  ASSERT_TRUE(predictor.recordAccepted(pose(0.0), 100 * kSecondNs));
  result = predictor.predict(pose(0.0), 100 * kSecondNs + 100000000LL);
  EXPECT_EQ(result.status, ConstantVelocityPredictionStatus::kNotReady);

  ASSERT_TRUE(predictor.recordAccepted(
      pose(0.1), 100 * kSecondNs + 100000000LL));
  EXPECT_TRUE(predictor.ready());
}

TEST(ConstantVelocityPredictor, AppliesOneAcceptedPoseDelta)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0), 100 * kSecondNs);
  predictor.recordAccepted(pose(0.1), 100 * kSecondNs + 100000000LL);

  const auto result = predictor.predict(
    pose(0.1), 100 * kSecondNs + 200000000LL);

  ASSERT_TRUE(result.applied);
  EXPECT_EQ(result.status, ConstantVelocityPredictionStatus::kApplied);
  EXPECT_NEAR(result.map_T_base.translation().x(), 0.2, 1.0e-9);
  EXPECT_NEAR(result.history_gap_s, 0.1, 1.0e-12);
  EXPECT_NEAR(result.prediction_gap_s, 0.1, 1.0e-12);
  EXPECT_FALSE(result.translation_clamped);
  EXPECT_FALSE(result.rotation_clamped);
}

TEST(ConstantVelocityPredictor, ClampsTranslationAndRotationLikeIndoorFrontend)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0, 0.0), 100 * kSecondNs);
  predictor.recordAccepted(
    pose(0.6, 30.0), 100 * kSecondNs + 100000000LL);

  const auto result = predictor.predict(
    pose(0.6, 30.0), 100 * kSecondNs + 200000000LL);

  ASSERT_TRUE(result.applied);
  EXPECT_TRUE(result.translation_clamped);
  EXPECT_TRUE(result.rotation_clamped);
  EXPECT_NEAR(result.raw_translation_m, 0.6, 1.0e-9);
  EXPECT_NEAR(result.raw_rotation_deg, 30.0, 1.0e-9);
  EXPECT_NEAR(result.applied_translation_m, 0.30, 1.0e-9);
  EXPECT_NEAR(result.applied_rotation_deg, 10.0, 1.0e-9);
  EXPECT_NEAR(rotationAngleDeg(result.map_T_base.linear()), 40.0, 1.0e-9);
}

TEST(ConstantVelocityPredictor, AppliesBodyFrameDeltaAfterRotatedPose)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0, 90.0), 100 * kSecondNs);
  predictor.recordAccepted(
    pose(0.1, 90.0), 100 * kSecondNs + 100000000LL);

  const auto result = predictor.predict(
    pose(0.1, 90.0), 100 * kSecondNs + 200000000LL);

  ASSERT_TRUE(result.applied);
  EXPECT_NEAR(result.map_T_base.translation().x(), 0.2, 1.0e-9);
  EXPECT_NEAR(result.map_T_base.translation().y(), 0.0, 1.0e-9);
  EXPECT_NEAR(rotationAngleDeg(result.map_T_base.linear()), 90.0, 1.0e-9);
}

TEST(ConstantVelocityPredictor, PreservesFullSe3Delta)
{
  ConstantVelocityPredictor predictor(1.0, 30.0, 0.50);
  const Eigen::Isometry3d previous = poseXyzRpy(
    Eigen::Vector3d(1.0, -2.0, 0.5), Eigen::Vector3d(5.0, -8.0, 35.0));
  const Eigen::Isometry3d step = poseXyzRpy(
    Eigen::Vector3d(0.12, -0.03, 0.04), Eigen::Vector3d(3.0, -2.0, 4.0));
  const Eigen::Isometry3d last = previous * step;
  predictor.recordAccepted(previous, 100 * kSecondNs);
  predictor.recordAccepted(last, 100 * kSecondNs + 100000000LL);

  const auto result = predictor.predict(
    last, 100 * kSecondNs + 200000000LL);

  ASSERT_TRUE(result.applied);
  EXPECT_TRUE(result.map_T_base.matrix().isApprox((last * step).matrix(), 1.0e-9));
}

TEST(ConstantVelocityPredictor, RejectedScansDoNotAdvanceAcceptedHistory)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0), 100 * kSecondNs);
  predictor.recordAccepted(pose(0.1), 100 * kSecondNs + 100000000LL);

  const auto first_attempt = predictor.predict(
    pose(0.1), 100 * kSecondNs + 200000000LL);
  const auto retry_after_reject = predictor.predict(
    pose(0.1), 100 * kSecondNs + 300000000LL);

  ASSERT_TRUE(first_attempt.applied);
  ASSERT_TRUE(retry_after_reject.applied);
  EXPECT_NEAR(
    retry_after_reject.map_T_base.translation().x(),
    first_attempt.map_T_base.translation().x(), 1.0e-9);
  EXPECT_NEAR(retry_after_reject.prediction_gap_s, 0.2, 1.0e-12);
}

TEST(ConstantVelocityPredictor, RebaseSeedsPoseWithoutInventingVelocity)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0), 100 * kSecondNs);
  predictor.recordAccepted(pose(0.1), 100 * kSecondNs + 100000000LL);
  ASSERT_TRUE(predictor.ready());

  ASSERT_TRUE(predictor.rebaseAccepted(
      pose(0.12), 100 * kSecondNs + 200000000LL));

  EXPECT_FALSE(predictor.ready());
  const auto result = predictor.predict(
    pose(0.12), 100 * kSecondNs + 300000000LL);
  EXPECT_EQ(result.status, ConstantVelocityPredictionStatus::kNotReady);
  EXPECT_FALSE(result.applied);
}

TEST(ConstantVelocityPredictor, RebasesAcrossAcceptedPoseGap)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0), 100 * kSecondNs);
  predictor.recordAccepted(pose(1.0), 100 * kSecondNs + 600000000LL);

  EXPECT_FALSE(predictor.ready());
  const auto result = predictor.predict(
    pose(1.0), 100 * kSecondNs + 700000000LL);
  EXPECT_EQ(result.status, ConstantVelocityPredictionStatus::kNotReady);
  EXPECT_FALSE(result.applied);
}

TEST(ConstantVelocityPredictor, SuppressesAndRebasesStalePrediction)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0), 100 * kSecondNs);
  predictor.recordAccepted(pose(0.1), 100 * kSecondNs + 100000000LL);

  const auto result = predictor.predict(
    pose(0.1), 100 * kSecondNs + 700000000LL);

  EXPECT_EQ(result.status, ConstantVelocityPredictionStatus::kPredictionGapTooLarge);
  EXPECT_FALSE(result.applied);
  EXPECT_FALSE(result.history_ready);
  EXPECT_FALSE(predictor.ready());
}

TEST(ConstantVelocityPredictor, AcceptsPredictionExactlyAtGapLimit)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0), 100 * kSecondNs);
  predictor.recordAccepted(pose(0.1), 100 * kSecondNs + 100000000LL);

  const auto result = predictor.predict(
    pose(0.1), 100 * kSecondNs + 600000000LL);

  EXPECT_EQ(result.status, ConstantVelocityPredictionStatus::kApplied);
  EXPECT_TRUE(result.applied);
  EXPECT_NEAR(result.prediction_gap_s, 0.5, 1.0e-12);
}

TEST(ConstantVelocityPredictor, NonmonotonicAcceptedStampStartsNewHistory)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0), 100 * kSecondNs);
  predictor.recordAccepted(pose(0.1), 100 * kSecondNs + 100000000LL);
  ASSERT_TRUE(predictor.ready());

  predictor.recordAccepted(pose(2.0), 10 * kSecondNs);

  EXPECT_FALSE(predictor.ready());
  const auto result = predictor.predict(pose(2.0), 10 * kSecondNs + 100000000LL);
  EXPECT_EQ(result.status, ConstantVelocityPredictionStatus::kNotReady);
}

TEST(ConstantVelocityPredictor, ResetAndInvalidPoseCannotCreatePrediction)
{
  ConstantVelocityPredictor predictor(0.30, 10.0, 0.50);
  predictor.recordAccepted(pose(0.0), 100 * kSecondNs);
  predictor.recordAccepted(pose(0.1), 100 * kSecondNs + 100000000LL);
  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(predictor.recordAccepted(invalid, 100 * kSecondNs + 200000000LL));
  EXPECT_TRUE(predictor.ready());

  const auto invalid_prediction = predictor.predict(
    invalid, 100 * kSecondNs + 200000000LL);
  EXPECT_EQ(
    invalid_prediction.status,
    ConstantVelocityPredictionStatus::kInvalidPose);
  EXPECT_FALSE(invalid_prediction.applied);

  predictor.reset();

  EXPECT_FALSE(predictor.ready());
  const auto result = predictor.predict(pose(0.1), 100 * kSecondNs + 300000000LL);
  EXPECT_EQ(result.status, ConstantVelocityPredictionStatus::kNotReady);
}

TEST(ConstantVelocityPredictor, RejectsUnboundedConfiguration)
{
  EXPECT_THROW(ConstantVelocityPredictor(0.0, 10.0, 0.50), std::invalid_argument);
  EXPECT_THROW(ConstantVelocityPredictor(0.30, 180.0, 0.50), std::invalid_argument);
  EXPECT_THROW(
    ConstantVelocityPredictor(
      0.30, 10.0, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}

}  // namespace
}  // namespace gole::slam_3d_localization
