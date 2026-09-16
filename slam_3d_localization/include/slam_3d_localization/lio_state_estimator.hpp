// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__LIO_STATE_ESTIMATOR_HPP_
#define SLAM_3D_LOCALIZATION__LIO_STATE_ESTIMATOR_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstdint>
#include <limits>
#include <string>

namespace gole::slam_3d_localization
{

struct ImuMeasurement
{
  // Measurements are expressed in the estimator body frame. The acceleration
  // is specific force (stationary, z-up body: approximately +gravity), not
  // gravity-removed linear acceleration.
  std::int64_t stamp_ns{0};
  Eigen::Vector3d angular_velocity_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration_mps2{Eigen::Vector3d::Zero()};
};

enum class ImuPropagationStatus
{
  kApplied,
  kNotInitialized,
  kNonmonotonicStamp,
  kGapTooLarge,
  kInvalidMeasurement,
  kNumericalFailure,
};

class LioStateEstimator
{
public:
  static constexpr int kStateDimension = 15;
  using Covariance = Eigen::Matrix<double, kStateDimension, kStateDimension>;

  struct Config
  {
    double gravity_mps2{9.80665};
    double max_imu_gap_s{0.05};
    double accelerometer_noise_mps2_sqrt_hz{0.15};
    double gyroscope_noise_radps_sqrt_hz{0.015};
    double accelerometer_bias_noise_mps3_sqrt_hz{0.001};
    double gyroscope_bias_noise_radps2_sqrt_hz{0.0001};
    double max_angular_velocity_radps{20.0};
    double max_linear_acceleration_mps2{100.0};
    double initial_position_std_m{0.05};
    double initial_velocity_std_mps{0.20};
    double initial_orientation_std_rad{0.05};
    double initial_accelerometer_bias_std_mps2{0.10};
    double initial_gyroscope_bias_std_radps{0.02};
    // Trust the incoming IMU by default: integrate its measurements without
    // estimating or subtracting gyro/accelerometer bias. Pose and velocity
    // still receive LiDAR (and optional wheel) corrections.
    bool imu_bias_estimation_enabled{false};
  };

  LioStateEstimator();
  explicit LioStateEstimator(const Config & config);

  static bool isValidConfig(const Config & config);
  static const char * statusName(ImuPropagationStatus status);

  void reset();
  bool initialize(
    const ImuMeasurement & measurement,
    const Eigen::Quaterniond & odom_q_body,
    const Eigen::Vector3d & gyroscope_bias_radps = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & accelerometer_bias_mps2 = Eigen::Vector3d::Zero(),
    const Eigen::Vector3d & initial_position_odom_m = Eigen::Vector3d::Zero());

  ImuPropagationStatus propagate(const ImuMeasurement & measurement);
  ImuPropagationStatus propagateTo(
    std::int64_t stamp_ns, const ImuMeasurement & held_measurement);
  bool resynchronizeAfterGap(
    const ImuMeasurement & measurement,
    double position_std_m,
    double velocity_std_mps,
    double orientation_std_rad);

  bool correctBodyPose(
    const Eigen::Isometry3d & measured_odom_T_body,
    double translation_std_m,
    double rotation_std_rad,
    double max_normalized_innovation_squared =
    std::numeric_limits<double>::infinity());
  bool correctBodyPose(
    const Eigen::Isometry3d & measured_odom_T_body,
    const Eigen::Matrix<double, 6, 6> & measurement_covariance,
    double max_normalized_innovation_squared =
    std::numeric_limits<double>::infinity());
  bool correctBodyForwardVelocity(double speed_mps, double standard_deviation_mps);

  bool initialized() const {return initialized_;}
  std::int64_t stampNs() const {return stamp_ns_;}
  // The body origin is the physical IMU origin. Its axes may first be mapped
  // into a configured base-aligned convention by the owning node.
  Eigen::Isometry3d odomTBody() const;
  const Eigen::Vector3d & position() const {return position_;}
  const Eigen::Vector3d & velocity() const {return velocity_;}
  const Eigen::Quaterniond & orientation() const {return orientation_;}
  const Eigen::Vector3d & gyroscopeBias() const {return gyroscope_bias_;}
  const Eigen::Vector3d & accelerometerBias() const {return accelerometer_bias_;}
  const Covariance & covariance() const {return covariance_;}
  const ImuMeasurement & lastMeasurement() const {return last_measurement_;}

private:
  using Vector15 = Eigen::Matrix<double, kStateDimension, 1>;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  using Matrix6x15 = Eigen::Matrix<double, 6, kStateDimension>;

  static Eigen::Matrix3d skew(const Eigen::Vector3d & vector);
  static Eigen::Quaterniond deltaQuaternion(const Eigen::Vector3d & rotation_vector);
  static Eigen::Vector3d rotationVector(const Eigen::Quaterniond & quaternion);
  void applyErrorState(const Vector15 & error_state);
  void enforceFixedImuBias();
  Covariance predictCovariance(
    const Eigen::Matrix3d & odom_R_base_mid,
    const Eigen::Vector3d & specific_force,
    const Eigen::Vector3d & angular_velocity, double dt) const;

  Config config_;
  bool initialized_{false};
  std::int64_t stamp_ns_{0};
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d gyroscope_bias_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_bias_{Eigen::Vector3d::Zero()};
  Covariance covariance_{Covariance::Identity()};
  ImuMeasurement last_measurement_;
};

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__LIO_STATE_ESTIMATOR_HPP_
