// Copyright 2026 Gole Robotics
#include "slam_3d_localization/constant_velocity_predictor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

std::int64_t secondsToNanoseconds(const double seconds)
{
  return static_cast<std::int64_t>(std::llround(seconds * 1.0e9));
}

Eigen::Isometry3d rigidPose(const Eigen::Isometry3d & pose)
{
  Eigen::Isometry3d rigid = Eigen::Isometry3d::Identity();
  rigid.translation() = pose.translation();
  rigid.linear() = Eigen::Quaterniond(pose.linear()).normalized().toRotationMatrix();
  return rigid;
}

double rotationAngleRad(const Eigen::Matrix3d & rotation)
{
  const Eigen::Quaterniond quaternion(rotation);
  return 2.0 * std::acos(std::clamp(std::abs(quaternion.normalized().w()), 0.0, 1.0));
}

}  // namespace

namespace gole::slam_3d_localization
{

ConstantVelocityPredictor::ConstantVelocityPredictor(
  const double translation_limit_m, const double rotation_limit_deg,
  const double max_gap_s)
: translation_limit_m_(translation_limit_m),
  rotation_limit_rad_(rotation_limit_deg / kRadiansToDegrees),
  max_gap_ns_(secondsToNanoseconds(max_gap_s))
{
  if (!std::isfinite(translation_limit_m) || translation_limit_m <= 0.0 ||
    !std::isfinite(rotation_limit_deg) || rotation_limit_deg <= 0.0 ||
    rotation_limit_deg >= 180.0 || !std::isfinite(max_gap_s) || max_gap_s <= 0.0)
  {
    throw std::invalid_argument("invalid constant-velocity prediction limits");
  }
}

void ConstantVelocityPredictor::reset()
{
  have_previous_ = false;
  have_last_ = false;
  previous_pose_.setIdentity();
  last_pose_.setIdentity();
  previous_stamp_ns_ = 0;
  last_stamp_ns_ = 0;
}

bool ConstantVelocityPredictor::rebaseAccepted(
  const Eigen::Isometry3d & map_T_base, const std::int64_t stamp_ns)
{
  if (!map_T_base.matrix().allFinite()) {
    return false;
  }
  previous_pose_.setIdentity();
  previous_stamp_ns_ = 0;
  have_previous_ = false;
  last_pose_ = rigidPose(map_T_base);
  last_stamp_ns_ = stamp_ns;
  have_last_ = true;
  return true;
}

bool ConstantVelocityPredictor::recordAccepted(
  const Eigen::Isometry3d & map_T_base, const std::int64_t stamp_ns)
{
  if (!map_T_base.matrix().allFinite()) {
    return false;
  }
  const Eigen::Isometry3d rigid_pose = rigidPose(map_T_base);
  if (!have_last_) {
    return rebaseAccepted(rigid_pose, stamp_ns);
  }

  const std::int64_t gap_ns = stamp_ns - last_stamp_ns_;
  if (gap_ns <= 0 || gap_ns > max_gap_ns_) {
    return rebaseAccepted(rigid_pose, stamp_ns);
  }

  previous_pose_ = last_pose_;
  previous_stamp_ns_ = last_stamp_ns_;
  last_pose_ = rigid_pose;
  last_stamp_ns_ = stamp_ns;
  have_previous_ = true;
  return true;
}

ConstantVelocityPredictionResult ConstantVelocityPredictor::predict(
  const Eigen::Isometry3d & fallback_map_T_base,
  const std::int64_t stamp_ns)
{
  ConstantVelocityPredictionResult result;
  if (!fallback_map_T_base.matrix().allFinite()) {
    result.status = ConstantVelocityPredictionStatus::kInvalidPose;
    return result;
  }
  result.map_T_base = rigidPose(fallback_map_T_base);
  result.history_ready = ready();
  if (!result.history_ready) {
    result.status = ConstantVelocityPredictionStatus::kNotReady;
    return result;
  }

  const std::int64_t history_gap_ns = last_stamp_ns_ - previous_stamp_ns_;
  const std::int64_t prediction_gap_ns = stamp_ns - last_stamp_ns_;
  result.history_gap_s = static_cast<double>(history_gap_ns) * 1.0e-9;
  result.prediction_gap_s = static_cast<double>(prediction_gap_ns) * 1.0e-9;
  if (history_gap_ns <= 0 || prediction_gap_ns <= 0) {
    result.status = ConstantVelocityPredictionStatus::kNonmonotonicStamp;
    rebaseToLast();
    result.history_ready = false;
    return result;
  }
  if (history_gap_ns > max_gap_ns_) {
    result.status = ConstantVelocityPredictionStatus::kHistoryGapTooLarge;
    rebaseToLast();
    result.history_ready = false;
    return result;
  }
  if (prediction_gap_ns > max_gap_ns_) {
    result.status = ConstantVelocityPredictionStatus::kPredictionGapTooLarge;
    rebaseToLast();
    result.history_ready = false;
    return result;
  }

  Eigen::Isometry3d delta = rigidPose(previous_pose_.inverse() * last_pose_);
  result.raw_translation_m = delta.translation().norm();
  result.raw_rotation_deg = rotationAngleRad(delta.linear()) * kRadiansToDegrees;

  if (result.raw_translation_m > translation_limit_m_) {
    delta.translation() *= translation_limit_m_ / result.raw_translation_m;
    result.translation_clamped = true;
  }
  if (result.raw_rotation_deg > rotation_limit_rad_ * kRadiansToDegrees) {
    const Eigen::AngleAxisd rotation(delta.linear());
    delta.linear() = Eigen::AngleAxisd(
      rotation_limit_rad_, rotation.axis()).toRotationMatrix();
    result.rotation_clamped = true;
  }
  delta = rigidPose(delta);
  result.applied_translation_m = delta.translation().norm();
  result.applied_rotation_deg = rotationAngleRad(delta.linear()) * kRadiansToDegrees;
  result.map_T_base = rigidPose(result.map_T_base * delta);
  result.status = ConstantVelocityPredictionStatus::kApplied;
  result.applied = true;
  return result;
}

bool ConstantVelocityPredictor::ready() const
{
  return have_previous_ && have_last_;
}

const char * ConstantVelocityPredictor::statusName(
  const ConstantVelocityPredictionStatus status)
{
  switch (status) {
    case ConstantVelocityPredictionStatus::kNotReady:
      return "not_ready";
    case ConstantVelocityPredictionStatus::kApplied:
      return "applied";
    case ConstantVelocityPredictionStatus::kNonmonotonicStamp:
      return "nonmonotonic_stamp";
    case ConstantVelocityPredictionStatus::kHistoryGapTooLarge:
      return "history_gap_too_large";
    case ConstantVelocityPredictionStatus::kPredictionGapTooLarge:
      return "prediction_gap_too_large";
    case ConstantVelocityPredictionStatus::kInvalidPose:
      return "invalid_pose";
    case ConstantVelocityPredictionStatus::kNotEvaluated:
    default:
      return "not_evaluated";
  }
}

void ConstantVelocityPredictor::rebaseToLast()
{
  have_previous_ = false;
  previous_pose_.setIdentity();
  previous_stamp_ns_ = 0;
}

}  // namespace gole::slam_3d_localization
