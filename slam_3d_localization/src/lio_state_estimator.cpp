// Copyright 2026 Gole Robotics
#include "slam_3d_localization/lio_state_estimator.hpp"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gole::slam_3d_localization
{

namespace
{

bool positiveFinite(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool validMeasurement(const ImuMeasurement & measurement)
{
  return measurement.stamp_ns >= 0 &&
         measurement.angular_velocity_radps.allFinite() &&
         measurement.linear_acceleration_mps2.allFinite();
}

}  // namespace

LioStateEstimator::LioStateEstimator()
: LioStateEstimator(Config{})
{
}

LioStateEstimator::LioStateEstimator(const Config & config)
: config_(config)
{
  if (!isValidConfig(config_)) {
    throw std::invalid_argument("invalid LIO state estimator configuration");
  }
  reset();
}

bool LioStateEstimator::isValidConfig(const Config & config)
{
  return positiveFinite(config.gravity_mps2) &&
         positiveFinite(config.max_imu_gap_s) &&
         positiveFinite(config.accelerometer_noise_mps2_sqrt_hz) &&
         positiveFinite(config.gyroscope_noise_radps_sqrt_hz) &&
         positiveFinite(config.accelerometer_bias_noise_mps3_sqrt_hz) &&
         positiveFinite(config.gyroscope_bias_noise_radps2_sqrt_hz) &&
         positiveFinite(config.max_angular_velocity_radps) &&
         positiveFinite(config.max_linear_acceleration_mps2) &&
         positiveFinite(config.initial_position_std_m) &&
         positiveFinite(config.initial_velocity_std_mps) &&
         positiveFinite(config.initial_orientation_std_rad) &&
         positiveFinite(config.initial_accelerometer_bias_std_mps2) &&
         positiveFinite(config.initial_gyroscope_bias_std_radps);
}

const char * LioStateEstimator::statusName(const ImuPropagationStatus status)
{
  switch (status) {
    case ImuPropagationStatus::kApplied:
      return "applied";
    case ImuPropagationStatus::kNotInitialized:
      return "not_initialized";
    case ImuPropagationStatus::kNonmonotonicStamp:
      return "nonmonotonic_stamp";
    case ImuPropagationStatus::kGapTooLarge:
      return "gap_too_large";
    case ImuPropagationStatus::kInvalidMeasurement:
      return "invalid_measurement";
    case ImuPropagationStatus::kNumericalFailure:
      return "numerical_failure";
  }
  return "unknown";
}

void LioStateEstimator::reset()
{
  initialized_ = false;
  stamp_ns_ = 0;
  position_.setZero();
  velocity_.setZero();
  orientation_.setIdentity();
  gyroscope_bias_.setZero();
  accelerometer_bias_.setZero();
  covariance_.setIdentity();
  enforceFixedImuBias();
  last_measurement_ = ImuMeasurement{};
}

bool LioStateEstimator::initialize(
  const ImuMeasurement & measurement,
  const Eigen::Quaterniond & odom_q_body,
  const Eigen::Vector3d & gyroscope_bias_radps,
  const Eigen::Vector3d & accelerometer_bias_mps2,
  const Eigen::Vector3d & initial_position_odom_m)
{
  if (!validMeasurement(measurement) || !odom_q_body.coeffs().allFinite() ||
    odom_q_body.norm() < 1.0e-9 || !gyroscope_bias_radps.allFinite() ||
    !accelerometer_bias_mps2.allFinite() || !initial_position_odom_m.allFinite())
  {
    return false;
  }

  position_ = initial_position_odom_m;
  velocity_.setZero();
  orientation_ = odom_q_body.normalized();
  if (config_.imu_bias_estimation_enabled) {
    gyroscope_bias_ = gyroscope_bias_radps;
    accelerometer_bias_ = accelerometer_bias_mps2;
  }
  covariance_.setZero();
  covariance_.diagonal().segment<3>(0).setConstant(
    config_.initial_position_std_m * config_.initial_position_std_m);
  covariance_.diagonal().segment<3>(3).setConstant(
    config_.initial_velocity_std_mps * config_.initial_velocity_std_mps);
  covariance_.diagonal().segment<3>(6).setConstant(
    config_.initial_orientation_std_rad * config_.initial_orientation_std_rad);
  if (config_.imu_bias_estimation_enabled) {
    covariance_.diagonal().segment<3>(9).setConstant(
      config_.initial_gyroscope_bias_std_radps *
      config_.initial_gyroscope_bias_std_radps);
    covariance_.diagonal().segment<3>(12).setConstant(
      config_.initial_accelerometer_bias_std_mps2 *
      config_.initial_accelerometer_bias_std_mps2);
  }
  enforceFixedImuBias();
  stamp_ns_ = measurement.stamp_ns;
  last_measurement_ = measurement;
  initialized_ = true;
  return true;
}

ImuPropagationStatus LioStateEstimator::propagate(const ImuMeasurement & measurement)
{
  if (!initialized_) {
    return ImuPropagationStatus::kNotInitialized;
  }
  if (!validMeasurement(measurement)) {
    return ImuPropagationStatus::kInvalidMeasurement;
  }
  if (measurement.angular_velocity_radps.norm() > config_.max_angular_velocity_radps ||
    measurement.linear_acceleration_mps2.norm() > config_.max_linear_acceleration_mps2)
  {
    return ImuPropagationStatus::kInvalidMeasurement;
  }
  if (measurement.stamp_ns <= stamp_ns_) {
    return ImuPropagationStatus::kNonmonotonicStamp;
  }
  const double dt = static_cast<double>(measurement.stamp_ns - stamp_ns_) * 1.0e-9;
  if (!std::isfinite(dt) || dt > config_.max_imu_gap_s) {
    return ImuPropagationStatus::kGapTooLarge;
  }

  const Eigen::Vector3d angular_velocity =
    0.5 * (last_measurement_.angular_velocity_radps +
    measurement.angular_velocity_radps) - gyroscope_bias_;
  const Eigen::Vector3d specific_force =
    0.5 * (last_measurement_.linear_acceleration_mps2 +
    measurement.linear_acceleration_mps2) - accelerometer_bias_;
  const Eigen::Quaterniond half_rotation = deltaQuaternion(0.5 * angular_velocity * dt);
  const Eigen::Matrix3d odom_R_base_mid = (orientation_ * half_rotation).toRotationMatrix();
  const Eigen::Vector3d gravity(0.0, 0.0, -config_.gravity_mps2);
  const Eigen::Vector3d acceleration_odom = odom_R_base_mid * specific_force + gravity;

  const Eigen::Vector3d candidate_position =
    position_ + velocity_ * dt + 0.5 * acceleration_odom * dt * dt;
  const Eigen::Vector3d candidate_velocity = velocity_ + acceleration_odom * dt;
  const Eigen::Quaterniond candidate_orientation =
    (orientation_ * deltaQuaternion(angular_velocity * dt)).normalized();

  Covariance dynamics = Covariance::Zero();
  dynamics.block<3, 3>(0, 3).setIdentity();
  dynamics.block<3, 3>(3, 6) = -odom_R_base_mid * skew(specific_force);
  dynamics.block<3, 3>(6, 6) = -skew(angular_velocity);
  if (config_.imu_bias_estimation_enabled) {
    dynamics.block<3, 3>(3, 12) = -odom_R_base_mid;
    dynamics.block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity();
  }
  const Covariance transition = Covariance::Identity() + dynamics * dt;

  Eigen::Matrix<double, kStateDimension, 12> noise_jacobian =
    Eigen::Matrix<double, kStateDimension, 12>::Zero();
  noise_jacobian.block<3, 3>(3, 0) = odom_R_base_mid;
  noise_jacobian.block<3, 3>(6, 3).setIdentity();
  if (config_.imu_bias_estimation_enabled) {
    noise_jacobian.block<3, 3>(9, 6).setIdentity();
    noise_jacobian.block<3, 3>(12, 9).setIdentity();
  }
  Eigen::Matrix<double, 12, 12> noise = Eigen::Matrix<double, 12, 12>::Zero();
  noise.diagonal().segment<3>(0).setConstant(
    config_.accelerometer_noise_mps2_sqrt_hz *
    config_.accelerometer_noise_mps2_sqrt_hz);
  noise.diagonal().segment<3>(3).setConstant(
    config_.gyroscope_noise_radps_sqrt_hz * config_.gyroscope_noise_radps_sqrt_hz);
  if (config_.imu_bias_estimation_enabled) {
    noise.diagonal().segment<3>(6).setConstant(
      config_.gyroscope_bias_noise_radps2_sqrt_hz *
      config_.gyroscope_bias_noise_radps2_sqrt_hz);
    noise.diagonal().segment<3>(9).setConstant(
      config_.accelerometer_bias_noise_mps3_sqrt_hz *
      config_.accelerometer_bias_noise_mps3_sqrt_hz);
  }
  Covariance candidate_covariance = transition * covariance_ * transition.transpose() +
    noise_jacobian * noise * noise_jacobian.transpose() * dt;
  candidate_covariance = 0.5 * (candidate_covariance + candidate_covariance.transpose());
  if (!candidate_position.allFinite() || !candidate_velocity.allFinite() ||
    !candidate_orientation.coeffs().allFinite() || candidate_orientation.norm() < 1.0e-9 ||
    !candidate_covariance.allFinite() ||
    (candidate_covariance.diagonal().array() < -1.0e-10).any())
  {
    return ImuPropagationStatus::kNumericalFailure;
  }

  position_ = candidate_position;
  velocity_ = candidate_velocity;
  orientation_ = candidate_orientation;
  covariance_ = candidate_covariance;
  enforceFixedImuBias();

  stamp_ns_ = measurement.stamp_ns;
  last_measurement_ = measurement;
  return ImuPropagationStatus::kApplied;
}

ImuPropagationStatus LioStateEstimator::propagateTo(
  const std::int64_t stamp_ns, const ImuMeasurement & held_measurement)
{
  ImuMeasurement target = held_measurement;
  target.stamp_ns = stamp_ns;
  return propagate(target);
}

bool LioStateEstimator::resynchronizeAfterGap(
  const ImuMeasurement & measurement,
  const double position_std_m,
  const double velocity_std_mps,
  const double orientation_std_rad)
{
  if (!initialized_ || !validMeasurement(measurement) ||
    measurement.stamp_ns <= stamp_ns_ || !positiveFinite(position_std_m) ||
    !positiveFinite(velocity_std_mps) || !positiveFinite(orientation_std_rad))
  {
    return false;
  }

  Covariance candidate_covariance = covariance_;
  candidate_covariance.diagonal().segment<3>(0).array() +=
    position_std_m * position_std_m;
  candidate_covariance.diagonal().segment<3>(3).array() +=
    velocity_std_mps * velocity_std_mps;
  candidate_covariance.diagonal().segment<3>(6).array() +=
    orientation_std_rad * orientation_std_rad;
  candidate_covariance = 0.5 * (candidate_covariance + candidate_covariance.transpose());
  if (!candidate_covariance.allFinite()) {
    return false;
  }
  covariance_ = candidate_covariance;
  enforceFixedImuBias();
  stamp_ns_ = measurement.stamp_ns;
  last_measurement_ = measurement;
  return true;
}

bool LioStateEstimator::correctBodyPose(
  const Eigen::Isometry3d & measured_odom_T_body,
  const double translation_std_m,
  const double rotation_std_rad,
  const double max_normalized_innovation_squared)
{
  if (!positiveFinite(translation_std_m) || !positiveFinite(rotation_std_rad)) {
    return false;
  }
  Matrix6 measurement_covariance = Matrix6::Zero();
  measurement_covariance.diagonal().head<3>().setConstant(
    translation_std_m * translation_std_m);
  measurement_covariance.diagonal().tail<3>().setConstant(
    rotation_std_rad * rotation_std_rad);
  return correctBodyPose(
    measured_odom_T_body, measurement_covariance,
    max_normalized_innovation_squared);
}

bool LioStateEstimator::correctBodyPose(
  const Eigen::Isometry3d & measured_odom_T_body,
  const Eigen::Matrix<double, 6, 6> & measurement_covariance,
  const double max_normalized_innovation_squared)
{
  if (!initialized_ || !measured_odom_T_body.matrix().allFinite() ||
    !measurement_covariance.allFinite() ||
    !measurement_covariance.isApprox(measurement_covariance.transpose(), 1.0e-9) ||
    (!positiveFinite(max_normalized_innovation_squared) &&
    !std::isinf(max_normalized_innovation_squared)))
  {
    return false;
  }
  const Eigen::LDLT<Matrix6> measurement_decomposition(measurement_covariance);
  if (measurement_decomposition.info() != Eigen::Success ||
    (measurement_decomposition.vectorD().array() <= 0.0).any())
  {
    return false;
  }

  const Eigen::Matrix3d measured_rotation = measured_odom_T_body.linear();
  if (std::abs(measured_rotation.determinant() - 1.0) > 1.0e-3 ||
    (measured_rotation.transpose() * measured_rotation - Eigen::Matrix3d::Identity()).norm() >
    1.0e-3)
  {
    return false;
  }

  Eigen::Matrix<double, 6, 1> innovation;
  innovation.head<3>() = measured_odom_T_body.translation() - position_;
  const Eigen::Quaterniond measured_orientation(measured_rotation);
  innovation.tail<3>() = rotationVector(orientation_.conjugate() * measured_orientation);

  Matrix6x15 observation = Matrix6x15::Zero();
  observation.block<3, 3>(0, 0).setIdentity();
  observation.block<3, 3>(3, 6).setIdentity();
  const Matrix6 innovation_covariance =
    observation * covariance_ * observation.transpose() + measurement_covariance;
  const Eigen::LDLT<Matrix6> decomposition(innovation_covariance);
  if (decomposition.info() != Eigen::Success) {
    return false;
  }
  const Eigen::Matrix<double, 6, 1> whitened_innovation =
    decomposition.solve(innovation);
  const double normalized_innovation_squared = innovation.dot(whitened_innovation);
  if (!std::isfinite(normalized_innovation_squared) ||
    normalized_innovation_squared > max_normalized_innovation_squared)
  {
    return false;
  }
  const Eigen::Matrix<double, kStateDimension, 6> gain =
    covariance_ * observation.transpose() * decomposition.solve(Matrix6::Identity());
  const Vector15 error_state = gain * innovation;
  if (!error_state.allFinite()) {
    return false;
  }
  const Eigen::Vector3d previous_position = position_;
  const Eigen::Vector3d previous_velocity = velocity_;
  const Eigen::Quaterniond previous_orientation = orientation_;
  const Eigen::Vector3d previous_gyroscope_bias = gyroscope_bias_;
  const Eigen::Vector3d previous_accelerometer_bias = accelerometer_bias_;
  const Covariance previous_covariance = covariance_;
  applyErrorState(error_state);
  const Covariance identity = Covariance::Identity();
  const Covariance residual_projection = identity - gain * observation;
  covariance_ = residual_projection * covariance_ * residual_projection.transpose() +
    gain * measurement_covariance * gain.transpose();
  Covariance reset_jacobian = Covariance::Identity();
  reset_jacobian.block<3, 3>(6, 6) =
    Eigen::Matrix3d::Identity() - 0.5 * skew(error_state.segment<3>(6));
  covariance_ = reset_jacobian * covariance_ * reset_jacobian.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  enforceFixedImuBias();
  if (!position_.allFinite() || !velocity_.allFinite() ||
    !orientation_.coeffs().allFinite() || orientation_.norm() < 1.0e-9 ||
    !gyroscope_bias_.allFinite() || !accelerometer_bias_.allFinite() ||
    !covariance_.allFinite() || (covariance_.diagonal().array() < -1.0e-10).any())
  {
    position_ = previous_position;
    velocity_ = previous_velocity;
    orientation_ = previous_orientation;
    gyroscope_bias_ = previous_gyroscope_bias;
    accelerometer_bias_ = previous_accelerometer_bias;
    covariance_ = previous_covariance;
    return false;
  }
  return true;
}

bool LioStateEstimator::correctBodyForwardVelocity(
  const double speed_mps, const double standard_deviation_mps)
{
  if (!initialized_ || !std::isfinite(speed_mps) ||
    !positiveFinite(standard_deviation_mps))
  {
    return false;
  }
  const Eigen::Vector3d velocity_base = orientation_.conjugate() * velocity_;
  Eigen::Matrix<double, 1, kStateDimension> observation =
    Eigen::Matrix<double, 1, kStateDimension>::Zero();
  observation.block<1, 3>(0, 3) =
    Eigen::RowVector3d::UnitX() * orientation_.conjugate().toRotationMatrix();
  observation.block<1, 3>(0, 6) =
    Eigen::RowVector3d::UnitX() * skew(velocity_base);
  const double innovation_covariance =
    (observation * covariance_ * observation.transpose())(0, 0) +
    standard_deviation_mps * standard_deviation_mps;
  if (!positiveFinite(innovation_covariance)) {
    return false;
  }
  const Eigen::Matrix<double, kStateDimension, 1> gain =
    covariance_ * observation.transpose() / innovation_covariance;
  const Vector15 error_state = gain * (speed_mps - velocity_base.x());
  if (!error_state.allFinite()) {
    return false;
  }
  const Eigen::Vector3d previous_position = position_;
  const Eigen::Vector3d previous_velocity = velocity_;
  const Eigen::Quaterniond previous_orientation = orientation_;
  const Eigen::Vector3d previous_gyroscope_bias = gyroscope_bias_;
  const Eigen::Vector3d previous_accelerometer_bias = accelerometer_bias_;
  const Covariance previous_covariance = covariance_;
  applyErrorState(error_state);
  const Covariance identity = Covariance::Identity();
  const Covariance residual_projection = identity - gain * observation;
  covariance_ = residual_projection * covariance_ * residual_projection.transpose() +
    gain * (standard_deviation_mps * standard_deviation_mps) * gain.transpose();
  Covariance reset_jacobian = Covariance::Identity();
  reset_jacobian.block<3, 3>(6, 6) =
    Eigen::Matrix3d::Identity() - 0.5 * skew(error_state.segment<3>(6));
  covariance_ = reset_jacobian * covariance_ * reset_jacobian.transpose();
  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  enforceFixedImuBias();
  if (!position_.allFinite() || !velocity_.allFinite() ||
    !orientation_.coeffs().allFinite() || orientation_.norm() < 1.0e-9 ||
    !gyroscope_bias_.allFinite() || !accelerometer_bias_.allFinite() ||
    !covariance_.allFinite() || (covariance_.diagonal().array() < -1.0e-10).any())
  {
    position_ = previous_position;
    velocity_ = previous_velocity;
    orientation_ = previous_orientation;
    gyroscope_bias_ = previous_gyroscope_bias;
    accelerometer_bias_ = previous_accelerometer_bias;
    covariance_ = previous_covariance;
    return false;
  }
  return true;
}

Eigen::Isometry3d LioStateEstimator::odomTBody() const
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = position_;
  result.linear() = orientation_.normalized().toRotationMatrix();
  return result;
}

Eigen::Matrix3d LioStateEstimator::skew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d result;
  result <<
    0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return result;
}

Eigen::Quaterniond LioStateEstimator::deltaQuaternion(
  const Eigen::Vector3d & rotation_vector)
{
  const double angle = rotation_vector.norm();
  if (angle < 1.0e-10) {
    Eigen::Quaterniond result(
      1.0, 0.5 * rotation_vector.x(), 0.5 * rotation_vector.y(),
      0.5 * rotation_vector.z());
    return result.normalized();
  }
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotation_vector / angle));
}

Eigen::Vector3d LioStateEstimator::rotationVector(const Eigen::Quaterniond & quaternion)
{
  Eigen::Quaterniond normalized = quaternion.normalized();
  if (normalized.w() < 0.0) {
    normalized.coeffs() *= -1.0;
  }
  const Eigen::Vector3d vector = normalized.vec();
  const double vector_norm = vector.norm();
  if (vector_norm < 1.0e-10) {
    return 2.0 * vector;
  }
  const double angle = 2.0 * std::atan2(vector_norm, normalized.w());
  return angle * vector / vector_norm;
}

void LioStateEstimator::applyErrorState(const Vector15 & error_state)
{
  position_ += error_state.segment<3>(0);
  velocity_ += error_state.segment<3>(3);
  orientation_ = (orientation_ * deltaQuaternion(error_state.segment<3>(6))).normalized();
  if (config_.imu_bias_estimation_enabled) {
    gyroscope_bias_ += error_state.segment<3>(9);
    accelerometer_bias_ += error_state.segment<3>(12);
  }
}

void LioStateEstimator::enforceFixedImuBias()
{
  if (config_.imu_bias_estimation_enabled) {
    return;
  }
  // Keep the existing 15-state API while making the six bias states fixed
  // constants. Zero cross-covariance prevents any measurement update from
  // implicitly reintroducing IMU bias estimation.
  gyroscope_bias_.setZero();
  accelerometer_bias_.setZero();
  covariance_.bottomRows<6>().setZero();
  covariance_.rightCols<6>().setZero();
}

}  // namespace gole::slam_3d_localization
