// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__IMU_INITIALIZER_HPP_
#define SLAM_3D_LOCALIZATION__IMU_INITIALIZER_HPP_

#include "slam_3d_localization/lio_state_estimator.hpp"

#include <Eigen/Geometry>

#include <cstddef>
#include <deque>
#include <string>

namespace gole::slam_3d_localization
{

struct ImuInitializationConfig
{
  // Keep gravity alignment/stationarity checks, but trust raw IMU by default.
  bool imu_bias_estimation_enabled{false};
  std::size_t min_samples{100};
  double min_duration_s{0.5};
  double min_acceleration_norm_mps2{8.5};
  double max_acceleration_norm_mps2{11.0};
  double max_acceleration_norm_std_mps2{0.35};
  double max_gyroscope_mean_norm_radps{0.05};
  double max_gyroscope_residual_rms_radps{0.05};
  double gravity_mps2{9.80665};
};

struct ImuInitializationResult
{
  bool evaluated{false};
  bool valid{false};
  std::string reason{"collecting"};
  std::size_t sample_count{0};
  double duration_s{0.0};
  Eigen::Quaterniond odom_q_base{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d gyroscope_bias_radps{Eigen::Vector3d::Zero()};
  Eigen::Vector3d accelerometer_bias_mps2{Eigen::Vector3d::Zero()};
  double mean_acceleration_norm_mps2{0.0};
  double acceleration_norm_std_mps2{0.0};
  double gyroscope_residual_rms_radps{0.0};
};

class StationaryImuInitializer
{
public:
  explicit StationaryImuInitializer(const ImuInitializationConfig & config);

  static bool isValidConfig(const ImuInitializationConfig & config);
  void reset();
  bool add(const ImuMeasurement & measurement);
  ImuInitializationResult evaluate() const;
  std::size_t size() const {return samples_.size();}

private:
  ImuInitializationConfig config_;
  std::deque<ImuMeasurement> samples_;
};

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__IMU_INITIALIZER_HPP_
