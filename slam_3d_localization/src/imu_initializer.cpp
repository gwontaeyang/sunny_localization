// Copyright 2026 Gole Robotics
#include "slam_3d_localization/imu_initializer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace gole::slam_3d_localization
{

namespace
{

bool positiveFinite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

StationaryImuInitializer::StationaryImuInitializer(
  const ImuInitializationConfig & config)
: config_(config)
{
  if (!isValidConfig(config_)) {
    throw std::invalid_argument("invalid stationary IMU initialization configuration");
  }
}

bool StationaryImuInitializer::isValidConfig(const ImuInitializationConfig & config)
{
  return config.min_samples >= 2U && positiveFinite(config.min_duration_s) &&
         positiveFinite(config.min_acceleration_norm_mps2) &&
         positiveFinite(config.max_acceleration_norm_mps2) &&
         config.min_acceleration_norm_mps2 < config.max_acceleration_norm_mps2 &&
         positiveFinite(config.max_acceleration_norm_std_mps2) &&
         positiveFinite(config.max_gyroscope_mean_norm_radps) &&
         positiveFinite(config.max_gyroscope_residual_rms_radps) &&
         positiveFinite(config.gravity_mps2);
}

void StationaryImuInitializer::reset()
{
  samples_.clear();
}

bool StationaryImuInitializer::add(const ImuMeasurement & measurement)
{
  if (measurement.stamp_ns < 0 || !measurement.angular_velocity_radps.allFinite() ||
    !measurement.linear_acceleration_mps2.allFinite())
  {
    return false;
  }
  if (!samples_.empty() && measurement.stamp_ns <= samples_.back().stamp_ns) {
    samples_.clear();
  }
  samples_.push_back(measurement);
  const std::size_t maximum_samples = std::max<std::size_t>(config_.min_samples * 4U, 400U);
  while (samples_.size() > maximum_samples) {
    samples_.pop_front();
  }
  return true;
}

ImuInitializationResult StationaryImuInitializer::evaluate() const
{
  ImuInitializationResult result;
  result.sample_count = samples_.size();
  if (samples_.size() < config_.min_samples) {
    return result;
  }
  result.duration_s = static_cast<double>(
    samples_.back().stamp_ns - samples_.front().stamp_ns) * 1.0e-9;
  if (!std::isfinite(result.duration_s) || result.duration_s < config_.min_duration_s) {
    return result;
  }
  result.evaluated = true;

  Eigen::Vector3d mean_gyroscope = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_acceleration = Eigen::Vector3d::Zero();
  std::vector<double> acceleration_norms;
  acceleration_norms.reserve(samples_.size());
  for (const auto & sample : samples_) {
    mean_gyroscope += sample.angular_velocity_radps;
    mean_acceleration += sample.linear_acceleration_mps2;
    acceleration_norms.push_back(sample.linear_acceleration_mps2.norm());
  }
  const double denominator = static_cast<double>(samples_.size());
  mean_gyroscope /= denominator;
  mean_acceleration /= denominator;
  if (config_.imu_bias_estimation_enabled) {
    result.gyroscope_bias_radps = mean_gyroscope;
  }
  result.mean_acceleration_norm_mps2 = mean_acceleration.norm();

  double acceleration_norm_variance = 0.0;
  double gyroscope_residual_squared_sum = 0.0;
  for (std::size_t index = 0; index < samples_.size(); ++index) {
    const double norm_error = acceleration_norms[index] -
      result.mean_acceleration_norm_mps2;
    acceleration_norm_variance += norm_error * norm_error;
    gyroscope_residual_squared_sum +=
      (samples_[index].angular_velocity_radps - mean_gyroscope).squaredNorm();
  }
  acceleration_norm_variance /= denominator;
  gyroscope_residual_squared_sum /= denominator;
  result.acceleration_norm_std_mps2 = std::sqrt(acceleration_norm_variance);
  result.gyroscope_residual_rms_radps = std::sqrt(gyroscope_residual_squared_sum);

  std::vector<std::string> reasons;
  if (result.mean_acceleration_norm_mps2 < config_.min_acceleration_norm_mps2 ||
    result.mean_acceleration_norm_mps2 > config_.max_acceleration_norm_mps2)
  {
    reasons.emplace_back("acceleration_norm_out_of_range");
  }
  if (result.acceleration_norm_std_mps2 > config_.max_acceleration_norm_std_mps2) {
    reasons.emplace_back("acceleration_unstable");
  }
  if (mean_gyroscope.norm() > config_.max_gyroscope_mean_norm_radps) {
    reasons.emplace_back("gyroscope_mean_too_large");
  }
  if (result.gyroscope_residual_rms_radps > config_.max_gyroscope_residual_rms_radps) {
    reasons.emplace_back("gyroscope_unstable");
  }
  if (!reasons.empty()) {
    result.reason.clear();
    for (std::size_t index = 0; index < reasons.size(); ++index) {
      if (index > 0U) {
        result.reason += ";";
      }
      result.reason += reasons[index];
    }
    return result;
  }

  const Eigen::Vector3d gravity_direction_base = mean_acceleration.normalized();
  result.odom_q_base = Eigen::Quaterniond::FromTwoVectors(
    gravity_direction_base, Eigen::Vector3d::UnitZ()).normalized();
  if (config_.imu_bias_estimation_enabled) {
    const Eigen::Vector3d expected_specific_force_base =
      result.odom_q_base.conjugate() *
      Eigen::Vector3d(0.0, 0.0, config_.gravity_mps2);
    result.accelerometer_bias_mps2 = mean_acceleration - expected_specific_force_base;
  }
  result.valid = result.odom_q_base.coeffs().allFinite() &&
    result.gyroscope_bias_radps.allFinite() && result.accelerometer_bias_mps2.allFinite();
  result.reason = result.valid ? "ok" : "nonfinite_initial_state";
  return result;
}

}  // namespace gole::slam_3d_localization
