// Copyright 2026 Gole Robotics
#include "slam_3d_localization/lio_odometry.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2/exceptions.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{

using sensor_msgs::msg::PointField;

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

const PointField * findField(
  const sensor_msgs::msg::PointCloud2 & message, const std::string & name)
{
  const auto iterator = std::find_if(
    message.fields.begin(), message.fields.end(),
    [&name](const PointField & field) {return field.name == name;});
  return iterator == message.fields.end() ? nullptr : &(*iterator);
}

std::size_t datatypeSize(const std::uint8_t datatype)
{
  switch (datatype) {
    case PointField::INT8:
    case PointField::UINT8:
      return 1U;
    case PointField::INT16:
    case PointField::UINT16:
      return 2U;
    case PointField::INT32:
    case PointField::UINT32:
    case PointField::FLOAT32:
      return 4U;
    case PointField::FLOAT64:
      return 8U;
    default:
      return 0U;
  }
}

template<typename T>
T readUnaligned(const std::uint8_t * data)
{
  T value{};
  std::memcpy(&value, data, sizeof(T));
  return value;
}

double readNumericField(const std::uint8_t * point, const PointField & field)
{
  const std::uint8_t * data = point + field.offset;
  switch (field.datatype) {
    case PointField::INT8:
      return static_cast<double>(readUnaligned<std::int8_t>(data));
    case PointField::UINT8:
      return static_cast<double>(readUnaligned<std::uint8_t>(data));
    case PointField::INT16:
      return static_cast<double>(readUnaligned<std::int16_t>(data));
    case PointField::UINT16:
      return static_cast<double>(readUnaligned<std::uint16_t>(data));
    case PointField::INT32:
      return static_cast<double>(readUnaligned<std::int32_t>(data));
    case PointField::UINT32:
      return static_cast<double>(readUnaligned<std::uint32_t>(data));
    case PointField::FLOAT32:
      return static_cast<double>(readUnaligned<float>(data));
    case PointField::FLOAT64:
      return readUnaligned<double>(data);
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}

std::int64_t rosStampNs(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

std::int64_t pointStampNs(
  const double value, const std::string & mode, const std::int64_t header_stamp_ns)
{
  if (!std::isfinite(value)) {
    return -1;
  }
  if (mode == "relative_nanoseconds") {
    const long double nanoseconds = static_cast<long double>(header_stamp_ns) +
      static_cast<long double>(value);
    if (value < 0.0 || nanoseconds < 0.0L ||
      nanoseconds > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
    {
      return -1;
    }
    return static_cast<std::int64_t>(std::llround(nanoseconds));
  }
  const bool absolute = mode == "absolute_seconds" ||
    (mode == "auto" && std::abs(value) > 1.0e8);
  const long double seconds = absolute ? static_cast<long double>(value) :
    static_cast<long double>(header_stamp_ns) * 1.0e-9L +
    static_cast<long double>(value);
  const long double nanoseconds = seconds * 1.0e9L;
  if (nanoseconds < 0.0L ||
    nanoseconds > static_cast<long double>(std::numeric_limits<std::int64_t>::max()))
  {
    return -1;
  }
  return static_cast<std::int64_t>(std::llround(nanoseconds));
}

Eigen::Isometry3d transformToIsometry(const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Quaterniond quaternion(
    transform.rotation.w, transform.rotation.x, transform.rotation.y,
    transform.rotation.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-9) {
    throw std::runtime_error("TF contains an invalid quaternion");
  }
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = quaternion.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(
    transform.translation.x, transform.translation.y, transform.translation.z);
  if (!result.matrix().allFinite()) {
    throw std::runtime_error("TF contains nonfinite values");
  }
  return result;
}

geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose result;
  const Eigen::Quaterniond quaternion(transform.linear());
  result.position.x = transform.translation().x();
  result.position.y = transform.translation().y();
  result.position.z = transform.translation().z();
  result.orientation.x = quaternion.x();
  result.orientation.y = quaternion.y();
  result.orientation.z = quaternion.z();
  result.orientation.w = quaternion.w();
  return result;
}

geometry_msgs::msg::Transform isometryToTransform(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Transform result;
  const Eigen::Quaterniond quaternion(transform.linear());
  result.translation.x = transform.translation().x();
  result.translation.y = transform.translation().y();
  result.translation.z = transform.translation().z();
  result.rotation.x = quaternion.x();
  result.rotation.y = quaternion.y();
  result.rotation.z = quaternion.z();
  result.rotation.w = quaternion.w();
  return result;
}

Eigen::Matrix3d rpyDegreesToRotation(const std::vector<double> & rpy_degrees)
{
  if (rpy_degrees.size() != 3U ||
    !std::all_of(
      rpy_degrees.begin(), rpy_degrees.end(), [](double value) {
        return std::isfinite(value);
      }))
  {
    throw std::invalid_argument("imu_to_base_rpy_deg must contain three finite values");
  }
  constexpr double kDegreesToRadians = 0.017453292519943295769;
  return (
    Eigen::AngleAxisd(rpy_degrees[2] * kDegreesToRadians, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy_degrees[1] * kDegreesToRadians, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy_degrees[0] * kDegreesToRadians, Eigen::Vector3d::UnitX()))
         .toRotationMatrix();
}

Eigen::Matrix3d skewMatrix(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d result;
  result <<
    0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return result;
}

}  // namespace

namespace gole::slam_3d_localization
{

LioOdometry::LioOdometry(const rclcpp::NodeOptions & options)
: Node("slam_3d_lio_odometry", options),
  imu_initializer_(initializer_config_),
  corrected_estimator_(state_config_),
  live_estimator_(state_config_)
{
  declareAndValidateParameters();
  imu_initializer_ = StationaryImuInitializer(initializer_config_);
  corrected_estimator_ = LioStateEstimator(state_config_);
  live_estimator_ = LioStateEstimator(state_config_);
  setupRosInterfaces();
  RCLCPP_INFO(
    get_logger(),
    "Odometry ready: mode=%s, cloud='%s', imu='%s', odom='%s', wheel_aiding=%s, imu_bias_estimation=%s",
    imu_enabled_ ? "lidar_imu" : "lidar_only (no deskew; scan-rate TF)",
    cloud_topic_.c_str(), imu_enabled_ ? imu_topic_.c_str() : "disabled",
    odom_topic_.c_str(),
    wheel_correction_enabled_ ? "enabled" : "disabled",
    imu_enabled_ && state_config_.imu_bias_estimation_enabled ? "enabled" : "disabled (fixed zero)");
}

void LioOdometry::declareAndValidateParameters()
{
  cloud_topic_ = declare_parameter<std::string>(
    "cloud_topic", "/hesai/center/lidar_points");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
  wheel_topic_ = declare_parameter<std::string>(
    "wheel_topic", "/motor_driver_node/odom");
  odom_topic_ = declare_parameter<std::string>(
    "odom_topic", "/localization_3d/lio/odom");
  scan_odom_topic_ = declare_parameter<std::string>(
    "scan_odom_topic", "/localization_3d/lio/scan_odom");
  deskewed_cloud_topic_ = declare_parameter<std::string>(
    "deskewed_cloud_topic", "/localization_3d/lio/deskewed_points");
  diagnostics_topic_ = declare_parameter<std::string>(
    "diagnostics_topic", "/localization_3d/lio/diagnostics");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "cv7_link");
  point_time_field_ = declare_parameter<std::string>("point_time_field", "timestamp");
  point_time_mode_ = declare_parameter<std::string>("point_time_mode", "auto");
  registration_method_ = declare_parameter<std::string>(
    "registration_method", registration_method_);
  publish_tf_ = declare_parameter<bool>("publish_tf", publish_tf_);
  imu_enabled_ = declare_parameter<bool>("imu_enabled", imu_enabled_);
  state_config_.imu_bias_estimation_enabled = declare_parameter<bool>(
    "imu_bias_estimation_enabled", state_config_.imu_bias_estimation_enabled);
  initializer_config_.imu_bias_estimation_enabled = state_config_.imu_bias_estimation_enabled;
  wheel_correction_enabled_ = declare_parameter<bool>(
    "wheel_correction_enabled", wheel_correction_enabled_);
  require_point_time_ = declare_parameter<bool>("require_point_time", require_point_time_);
  cloud_subscription_reliable_ = declare_parameter<bool>(
    "cloud_subscription_reliable", cloud_subscription_reliable_);
  cloud_subscription_depth_ = declare_parameter<int>(
    "cloud_subscription_depth", cloud_subscription_depth_);
  pending_cloud_limit_ = declare_parameter<int>("pending_cloud_limit", pending_cloud_limit_);
  tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", tf_timeout_s_);
  imu_epoch_reset_threshold_s_ = declare_parameter<double>(
    "imu_epoch_reset_threshold_s", imu_epoch_reset_threshold_s_);
  imu_buffer_duration_s_ = declare_parameter<double>(
    "imu_buffer_duration_s", imu_buffer_duration_s_);
  max_scan_duration_s_ = declare_parameter<double>(
    "max_scan_duration_s", max_scan_duration_s_);
  point_time_header_tolerance_s_ = declare_parameter<double>(
    "point_time_header_tolerance_s", point_time_header_tolerance_s_);
  imu_gap_position_std_m_ = declare_parameter<double>(
    "imu_gap_position_std_m", imu_gap_position_std_m_);
  imu_gap_velocity_std_mps_ = declare_parameter<double>(
    "imu_gap_velocity_std_mps", imu_gap_velocity_std_mps_);
  imu_gap_orientation_std_deg_ = declare_parameter<double>(
    "imu_gap_orientation_std_deg", imu_gap_orientation_std_deg_);
  scan_voxel_leaf_m_ = declare_parameter<double>(
    "scan_voxel_leaf_m", scan_voxel_leaf_m_);
  submap_voxel_leaf_m_ = declare_parameter<double>(
    "submap_voxel_leaf_m", submap_voxel_leaf_m_);
  local_submap_radius_m_ = declare_parameter<double>(
    "local_submap_radius_m", local_submap_radius_m_);
  min_range_m_ = declare_parameter<double>("min_range_m", min_range_m_);
  max_range_m_ = declare_parameter<double>("max_range_m", max_range_m_);
  z_min_m_ = declare_parameter<double>("z_min_m", z_min_m_);
  z_max_m_ = declare_parameter<double>("z_max_m", z_max_m_);
  crop_box_enabled_ = declare_parameter<bool>("crop_box_enabled", crop_box_enabled_);
  const auto crop_min = declare_parameter<std::vector<double>>(
    "crop_min_xyz_m", {crop_min_.x(), crop_min_.y(), crop_min_.z()});
  const auto crop_max = declare_parameter<std::vector<double>>(
    "crop_max_xyz_m", {crop_max_.x(), crop_max_.y(), crop_max_.z()});
  if (crop_min.size() != 3U || crop_max.size() != 3U) {
    throw std::invalid_argument("crop_min_xyz_m and crop_max_xyz_m must contain three values");
  }
  crop_min_ = Eigen::Vector3d(crop_min[0], crop_min[1], crop_min[2]);
  crop_max_ = Eigen::Vector3d(crop_max[0], crop_max[1], crop_max[2]);
  min_scan_points_ = declare_parameter<int>("min_scan_points", min_scan_points_);
  min_submap_points_ = declare_parameter<int>("min_submap_points", min_submap_points_);
  max_keyframes_ = declare_parameter<int>("max_keyframes", max_keyframes_);
  keyframe_translation_m_ = declare_parameter<double>(
    "keyframe_translation_m", keyframe_translation_m_);
  keyframe_rotation_deg_ = declare_parameter<double>(
    "keyframe_rotation_deg", keyframe_rotation_deg_);
  gicp_max_iterations_ = declare_parameter<int>(
    "gicp_max_iterations", gicp_max_iterations_);
  gicp_max_correspondence_m_ = declare_parameter<double>(
    "gicp_max_correspondence_m", gicp_max_correspondence_m_);
  fitness_threshold_ = declare_parameter<double>("fitness_threshold", fitness_threshold_);
  max_correction_translation_m_ = declare_parameter<double>(
    "max_correction_translation_m", max_correction_translation_m_);
  max_correction_rotation_deg_ = declare_parameter<double>(
    "max_correction_rotation_deg", max_correction_rotation_deg_);
  lidar_translation_std_min_m_ = declare_parameter<double>(
    "lidar_translation_std_min_m", lidar_translation_std_min_m_);
  lidar_rotation_std_min_deg_ = declare_parameter<double>(
    "lidar_rotation_std_min_deg", lidar_rotation_std_min_deg_);
  lidar_noise_scale_ = declare_parameter<double>("lidar_noise_scale", lidar_noise_scale_);
  lidar_pose_nis_gate_ = declare_parameter<double>(
    "lidar_pose_nis_gate", lidar_pose_nis_gate_);
  lidar_weak_axis_noise_multiplier_ = declare_parameter<double>(
    "lidar_weak_axis_noise_multiplier", lidar_weak_axis_noise_multiplier_);
  wheel_max_age_s_ = declare_parameter<double>("wheel_max_age_s", wheel_max_age_s_);
  wheel_max_abs_speed_mps_ = declare_parameter<double>(
    "wheel_max_abs_speed_mps", wheel_max_abs_speed_mps_);
  wheel_max_innovation_mps_ = declare_parameter<double>(
    "wheel_max_innovation_mps", wheel_max_innovation_mps_);
  wheel_fallback_std_mps_ = declare_parameter<double>(
    "wheel_fallback_std_mps", wheel_fallback_std_mps_);
  base_R_imu_ = rpyDegreesToRotation(
    declare_parameter<std::vector<double>>("imu_to_base_rpy_deg", {0.0, 0.0, 0.0}));
  const auto base_p_imu = declare_parameter<std::vector<double>>(
    "imu_position_in_base_m", {0.0, 0.0, 0.0});
  if (base_p_imu.size() != 3U) {
    throw std::invalid_argument("imu_position_in_base_m must contain three values");
  }
  base_p_imu_ = Eigen::Vector3d(base_p_imu[0], base_p_imu[1], base_p_imu[2]);

  const int initialization_min_samples = declare_parameter<int>(
    "imu_initialization_min_samples", static_cast<int>(initializer_config_.min_samples));
  if (initialization_min_samples < 2) {
    throw std::invalid_argument("imu_initialization_min_samples must be at least two");
  }
  initializer_config_.min_samples = static_cast<std::size_t>(initialization_min_samples);
  initializer_config_.min_duration_s = declare_parameter<double>(
    "imu_initialization_min_duration_s", initializer_config_.min_duration_s);
  initializer_config_.min_acceleration_norm_mps2 = declare_parameter<double>(
    "imu_initialization_min_acceleration_norm_mps2",
    initializer_config_.min_acceleration_norm_mps2);
  initializer_config_.max_acceleration_norm_mps2 = declare_parameter<double>(
    "imu_initialization_max_acceleration_norm_mps2",
    initializer_config_.max_acceleration_norm_mps2);
  initializer_config_.max_acceleration_norm_std_mps2 = declare_parameter<double>(
    "imu_initialization_max_acceleration_norm_std_mps2",
    initializer_config_.max_acceleration_norm_std_mps2);
  initializer_config_.max_gyroscope_mean_norm_radps = declare_parameter<double>(
    "imu_initialization_max_gyroscope_mean_norm_radps",
    initializer_config_.max_gyroscope_mean_norm_radps);
  initializer_config_.max_gyroscope_residual_rms_radps = declare_parameter<double>(
    "imu_initialization_max_gyroscope_residual_rms_radps",
    initializer_config_.max_gyroscope_residual_rms_radps);

  state_config_.gravity_mps2 = declare_parameter<double>(
    "gravity_mps2", state_config_.gravity_mps2);
  initializer_config_.gravity_mps2 = state_config_.gravity_mps2;
  state_config_.max_imu_gap_s = declare_parameter<double>(
    "max_imu_gap_s", state_config_.max_imu_gap_s);
  state_config_.accelerometer_noise_mps2_sqrt_hz = declare_parameter<double>(
    "accelerometer_noise_mps2_sqrt_hz",
    state_config_.accelerometer_noise_mps2_sqrt_hz);
  state_config_.gyroscope_noise_radps_sqrt_hz = declare_parameter<double>(
    "gyroscope_noise_radps_sqrt_hz", state_config_.gyroscope_noise_radps_sqrt_hz);
  state_config_.accelerometer_bias_noise_mps3_sqrt_hz = declare_parameter<double>(
    "accelerometer_bias_noise_mps3_sqrt_hz",
    state_config_.accelerometer_bias_noise_mps3_sqrt_hz);
  state_config_.gyroscope_bias_noise_radps2_sqrt_hz = declare_parameter<double>(
    "gyroscope_bias_noise_radps2_sqrt_hz",
    state_config_.gyroscope_bias_noise_radps2_sqrt_hz);
  state_config_.max_angular_velocity_radps = declare_parameter<double>(
    "max_angular_velocity_radps", state_config_.max_angular_velocity_radps);
  state_config_.max_linear_acceleration_mps2 = declare_parameter<double>(
    "max_linear_acceleration_mps2", state_config_.max_linear_acceleration_mps2);
  state_config_.initial_position_std_m = declare_parameter<double>(
    "initial_position_std_m", state_config_.initial_position_std_m);
  state_config_.initial_velocity_std_mps = declare_parameter<double>(
    "initial_velocity_std_mps", state_config_.initial_velocity_std_mps);
  state_config_.initial_orientation_std_rad = declare_parameter<double>(
    "initial_orientation_std_rad", state_config_.initial_orientation_std_rad);
  state_config_.initial_accelerometer_bias_std_mps2 = declare_parameter<double>(
    "initial_accelerometer_bias_std_mps2",
    state_config_.initial_accelerometer_bias_std_mps2);
  state_config_.initial_gyroscope_bias_std_radps = declare_parameter<double>(
    "initial_gyroscope_bias_std_radps",
    state_config_.initial_gyroscope_bias_std_radps);

  const auto declare_positive_size = [this](const std::string & name, std::size_t value) {
      const int configured = declare_parameter<int>(name, static_cast<int>(value));
      if (configured <= 0) {
        throw std::invalid_argument(name + " must be positive");
      }
      return static_cast<std::size_t>(configured);
    };
  // Keep enough neighbours per surfel after coarse scan/submap downsampling.
  surfel_config_.voxel_size = declare_parameter<double>(
    "surfel_voxel_size_m", 2.0);
  surfel_config_.min_points_per_voxel = declare_positive_size(
    "surfel_min_points_per_voxel", 3U);
  surfel_config_.max_planarity_ratio = declare_parameter<double>(
    "surfel_max_planarity_ratio", surfel_config_.max_planarity_ratio);
  surfel_config_.min_tangent_variance = declare_parameter<double>(
    "surfel_min_tangent_variance", surfel_config_.min_tangent_variance);
  surfel_config_.max_plane_distance = declare_parameter<double>(
    "surfel_max_plane_distance_m", surfel_config_.max_plane_distance);
  surfel_config_.max_center_distance = declare_parameter<double>(
    "surfel_max_center_distance_m", 2.5);
  surfel_config_.min_correspondences = declare_positive_size(
    "surfel_min_correspondences", 40U);
  surfel_config_.min_correspondence_ratio = declare_parameter<double>(
    "surfel_min_correspondence_ratio", surfel_config_.min_correspondence_ratio);
  surfel_config_.huber_delta = declare_parameter<double>(
    "surfel_huber_delta_m", surfel_config_.huber_delta);
  surfel_config_.max_iterations = declare_parameter<int>(
    "surfel_max_iterations", 4);
  surfel_config_.max_backtracking_steps = declare_parameter<int>(
    "surfel_max_backtracking_steps", 2);
  surfel_config_.backtracking_factor = declare_parameter<double>(
    "surfel_backtracking_factor", surfel_config_.backtracking_factor);
  surfel_config_.rotation_convergence_rad = declare_parameter<double>(
    "surfel_rotation_convergence_rad", surfel_config_.rotation_convergence_rad);
  surfel_config_.translation_convergence_m = declare_parameter<double>(
    "surfel_translation_convergence_m", surfel_config_.translation_convergence_m);
  surfel_config_.relative_cost_convergence = declare_parameter<double>(
    "surfel_relative_cost_convergence", surfel_config_.relative_cost_convergence);
  surfel_config_.max_rotation_step_rad = declare_parameter<double>(
    "surfel_max_rotation_step_rad", surfel_config_.max_rotation_step_rad);
  surfel_config_.max_translation_step_m = declare_parameter<double>(
    "surfel_max_translation_step_m", surfel_config_.max_translation_step_m);
  surfel_config_.rotation_scale = declare_parameter<double>(
    "surfel_rotation_scale_m_per_rad", surfel_config_.rotation_scale);
  surfel_config_.relative_eigen_threshold = declare_parameter<double>(
    "surfel_relative_eigen_threshold", surfel_config_.relative_eigen_threshold);
  surfel_config_.min_observable_rank = declare_parameter<int>(
    "surfel_min_observable_rank", surfel_config_.min_observable_rank);
  surfel_config_.max_quality_rmse = declare_parameter<double>(
    "surfel_max_quality_rmse_m", surfel_config_.max_quality_rmse);
  surfel_config_.max_quality_p95 = declare_parameter<double>(
    "surfel_max_quality_p95_m", surfel_config_.max_quality_p95);

  const bool valid_strings = !cloud_topic_.empty() && !imu_topic_.empty() &&
    !odom_topic_.empty() && !scan_odom_topic_.empty() && scan_odom_topic_ != odom_topic_ &&
    !deskewed_cloud_topic_.empty() && !diagnostics_topic_.empty() &&
    !odom_frame_.empty() && !base_frame_.empty() && odom_frame_ != base_frame_ &&
    !imu_frame_.empty() && !point_time_field_.empty();
  const bool valid_modes =
    (point_time_mode_ == "auto" || point_time_mode_ == "absolute_seconds" ||
    point_time_mode_ == "relative_seconds" || point_time_mode_ == "relative_nanoseconds") &&
    (registration_method_ == "gicp" || registration_method_ == "voxel_surfel");
  const std::array<double, 27> scalar_values = {
    tf_timeout_s_, imu_epoch_reset_threshold_s_, imu_buffer_duration_s_,
    max_scan_duration_s_, point_time_header_tolerance_s_, imu_gap_position_std_m_,
    imu_gap_velocity_std_mps_, imu_gap_orientation_std_deg_, scan_voxel_leaf_m_,
    submap_voxel_leaf_m_, local_submap_radius_m_, min_range_m_, max_range_m_, z_min_m_,
    z_max_m_, keyframe_translation_m_, keyframe_rotation_deg_, gicp_max_correspondence_m_,
    fitness_threshold_, max_correction_translation_m_, max_correction_rotation_deg_,
    lidar_translation_std_min_m_, lidar_rotation_std_min_deg_, lidar_noise_scale_,
    lidar_pose_nis_gate_, lidar_weak_axis_noise_multiplier_, wheel_max_age_s_};
  const bool finite_scalars = std::all_of(
    scalar_values.begin(), scalar_values.end(),
    [](const double value) {return std::isfinite(value);}) &&
    std::isfinite(wheel_max_abs_speed_mps_) &&
    std::isfinite(wheel_max_innovation_mps_) &&
    std::isfinite(wheel_fallback_std_mps_);
  const bool valid_scalars = cloud_subscription_depth_ > 0 && pending_cloud_limit_ > 0 &&
    tf_timeout_s_ >= 0.0 && imu_epoch_reset_threshold_s_ > 0.0 &&
    imu_buffer_duration_s_ > max_scan_duration_s_ && max_scan_duration_s_ > 0.0 &&
    point_time_header_tolerance_s_ >= 0.0 && imu_gap_position_std_m_ > 0.0 &&
    imu_gap_velocity_std_mps_ > 0.0 && imu_gap_orientation_std_deg_ > 0.0 &&
    scan_voxel_leaf_m_ > 0.0 && submap_voxel_leaf_m_ > 0.0 &&
    local_submap_radius_m_ > 0.0 && min_range_m_ >= 0.0 && max_range_m_ > min_range_m_ &&
    z_max_m_ > z_min_m_ && crop_min_.allFinite() && crop_max_.allFinite() &&
    base_p_imu_.allFinite() &&
    (crop_min_.array() < crop_max_.array()).all() && min_scan_points_ > 0 &&
    min_submap_points_ > 0 && max_keyframes_ >= 2 && keyframe_translation_m_ > 0.0 &&
    keyframe_rotation_deg_ > 0.0 && gicp_max_iterations_ > 0 &&
    gicp_max_correspondence_m_ > 0.0 && fitness_threshold_ > 0.0 &&
    max_correction_translation_m_ > 0.0 && max_correction_rotation_deg_ > 0.0 &&
    lidar_translation_std_min_m_ > 0.0 && lidar_rotation_std_min_deg_ > 0.0 &&
    lidar_noise_scale_ > 0.0 && lidar_pose_nis_gate_ > 0.0 && wheel_max_age_s_ >= 0.0 &&
    lidar_weak_axis_noise_multiplier_ >= 1.0 &&
    wheel_max_abs_speed_mps_ > 0.0 && wheel_max_innovation_mps_ > 0.0 &&
    wheel_fallback_std_mps_ > 0.0;
  if (!imu_enabled_ && wheel_correction_enabled_) {
    throw std::invalid_argument("wheel_correction_enabled requires imu_enabled=true");
  }
  if (!valid_strings || !valid_modes || !finite_scalars || !valid_scalars ||
    !StationaryImuInitializer::isValidConfig(initializer_config_) ||
    !LioStateEstimator::isValidConfig(state_config_) ||
    !::gole::slam_3d_core::VoxelSurfelRegistration::isValidConfig(surfel_config_) ||
    (wheel_correction_enabled_ && wheel_topic_.empty()))
  {
    throw std::invalid_argument("invalid LIO parameters");
  }
}

void LioOdometry::setupRosInterfaces()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (imu_enabled_) {
    imu_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions imu_options;
    imu_options.callback_group = imu_callback_group_;
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS().keep_last(400),
      std::bind(&LioOdometry::imuCallback, this, std::placeholders::_1), imu_options);
  }

  cloud_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions cloud_options;
  cloud_options.callback_group = cloud_callback_group_;
  auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(cloud_subscription_depth_)).durability_volatile();
  if (cloud_subscription_reliable_) {
    cloud_qos.reliable();
  } else {
    cloud_qos.best_effort();
  }
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, cloud_qos,
    std::bind(&LioOdometry::cloudCallback, this, std::placeholders::_1), cloud_options);
  if (imu_enabled_) {
    cloud_processing_timer_ = create_wall_timer(
      std::chrono::milliseconds(2),
      std::bind(&LioOdometry::processPendingClouds, this), cloud_callback_group_);
  }

  if (wheel_correction_enabled_) {
    wheel_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions wheel_options;
    wheel_options.callback_group = wheel_callback_group_;
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_topic_, rclcpp::QoS(100).reliable(),
      std::bind(&LioOdometry::wheelCallback, this, std::placeholders::_1), wheel_options);
  }

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(100).reliable());
  scan_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    scan_odom_topic_, rclcpp::QoS(20).reliable());
  deskewed_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    deskewed_cloud_topic_, rclcpp::QoS(10).reliable());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    diagnostics_topic_, rclcpp::QoS(20));
}

std::optional<ImuMeasurement> LioOdometry::transformImuMeasurement(
  const sensor_msgs::msg::Imu & message) const
{
  if (message.header.frame_id != imu_frame_) {
    return std::nullopt;
  }
  ImuMeasurement measurement;
  measurement.stamp_ns = rosStampNs(message.header.stamp);
  measurement.angular_velocity_radps = base_R_imu_ * Eigen::Vector3d(
    message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z);
  measurement.linear_acceleration_mps2 = base_R_imu_ * Eigen::Vector3d(
    message.linear_acceleration.x, message.linear_acceleration.y,
    message.linear_acceleration.z);
  if (measurement.stamp_ns < 0 || !measurement.angular_velocity_radps.allFinite() ||
    !measurement.linear_acceleration_mps2.allFinite())
  {
    return std::nullopt;
  }
  return measurement;
}

void LioOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr message)
{
  if (!imu_enabled_) {
    return;
  }
  const auto measurement_optional = transformImuMeasurement(*message);
  if (!measurement_optional.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring IMU with wrong frame or nonfinite values; expected frame '%s'",
      imu_frame_.c_str());
    return;
  }
  const ImuMeasurement measurement = *measurement_optional;
  std::optional<LioStateEstimator> publish_state;
  std::uint64_t publish_revision = 0U;
  bool initialized_now = false;
  std::string initialization_failure;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const std::int64_t reset_threshold_ns = static_cast<std::int64_t>(
      std::llround(imu_epoch_reset_threshold_s_ * 1.0e9));
    if (latest_imu_stamp_ns_ > 0 &&
      measurement.stamp_ns < latest_imu_stamp_ns_ - reset_threshold_ns)
    {
      resetForTimeEpoch("imu_timestamp_rewind");
    } else if (latest_imu_stamp_ns_ > 0 && measurement.stamp_ns <= latest_imu_stamp_ns_) {
      return;
    }
    latest_imu_stamp_ns_ = measurement.stamp_ns;

    if (!corrected_estimator_.initialized()) {
      imu_initializer_.add(measurement);
      const auto initialization = imu_initializer_.evaluate();
      if (initialization.evaluated && initialization.valid) {
        if (!corrected_estimator_.initialize(
            measurement, initialization.odom_q_base,
            initialization.gyroscope_bias_radps,
            initialization.accelerometer_bias_mps2,
            initialization.odom_q_base * base_p_imu_))
        {
          initialization_failure = "state_initialization_failed";
          imu_initializer_.reset();
        } else {
          live_estimator_ = corrected_estimator_;
          imu_buffer_.clear();
          initialized_now = true;
          publish_state = live_estimator_;
          publish_revision = publication_revision_;
        }
      } else if (initialization.evaluated) {
        initialization_failure = initialization.reason;
        imu_initializer_.reset();
        imu_initializer_.add(measurement);
      }
    } else {
      imu_buffer_.push_back(measurement);
      const std::int64_t cutoff_ns = measurement.stamp_ns - static_cast<std::int64_t>(
        std::llround(imu_buffer_duration_s_ * 1.0e9));
      while (!imu_buffer_.empty() &&
        imu_buffer_.front().stamp_ns < cutoff_ns &&
        imu_buffer_.front().stamp_ns <= corrected_estimator_.stampNs())
      {
        imu_buffer_.pop_front();
      }
      const ImuPropagationStatus status = live_estimator_.propagate(measurement);
      last_propagation_status_ = LioStateEstimator::statusName(status);
      if (status == ImuPropagationStatus::kGapTooLarge ||
        status == ImuPropagationStatus::kNumericalFailure)
      {
        constexpr double kDegreesToRadians = 0.017453292519943295769;
        if (live_estimator_.resynchronizeAfterGap(
            measurement, imu_gap_position_std_m_, imu_gap_velocity_std_mps_,
            imu_gap_orientation_std_deg_ * kDegreesToRadians))
        {
          corrected_estimator_ = live_estimator_;
          imu_buffer_.clear();
          wheel_observations_.clear();
          pending_clouds_.clear();
          ++imu_gap_recovery_count_;
          ++state_revision_;
          ++publication_revision_;
          invalidateOlderPublications(publication_revision_);
          last_propagation_status_ = "gap_resynchronized";
          publish_state = live_estimator_;
          publish_revision = publication_revision_;
        } else {
          resetForTimeEpoch("imu_propagation_" + last_propagation_status_);
          latest_imu_stamp_ns_ = measurement.stamp_ns;
          imu_initializer_.add(measurement);
        }
      } else if (status == ImuPropagationStatus::kInvalidMeasurement) {
        imu_buffer_.pop_back();
        latest_imu_stamp_ns_ = live_estimator_.stampNs();
      } else if (status == ImuPropagationStatus::kApplied) {
        boundImuBuffer();
        publish_state = live_estimator_;
        publish_revision = publication_revision_;
      }
    }
  }
  if (publish_state.has_value()) {
    publishOdometry(
      *publish_state, rclcpp::Time(publish_state->stampNs(), RCL_ROS_TIME),
      publish_revision);
  }
  if (initialized_now) {
    publishDiagnostic(
      rclcpp::Time(measurement.stamp_ns, RCL_ROS_TIME),
      diagnostic_msgs::msg::DiagnosticStatus::OK, "initializing", "imu_initialized");
  } else if (!initialization_failure.empty()) {
    publishDiagnostic(
      rclcpp::Time(measurement.stamp_ns, RCL_ROS_TIME),
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "initializing",
      initialization_failure);
  }
}

void LioOdometry::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
{
  PendingCloud pending;
  pending.message = message;
  if (!imu_enabled_) {
    // A rigid, non-deskewed scan has one representative header timestamp.
    // Do not inspect point times or wait for inertial initialization/coverage.
    pending.start_stamp_ns = rosStampNs(message->header.stamp);
    pending.end_stamp_ns = pending.start_stamp_ns;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto reset_ns = static_cast<std::int64_t>(imu_epoch_reset_threshold_s_ * 1.0e9);
      if (pending.end_stamp_ns >= 0 && lidar_only_input_stamp_ns_ >= 0 &&
        pending.end_stamp_ns < lidar_only_input_stamp_ns_ - reset_ns)
      {
        resetForTimeEpoch("lidar_timestamp_epoch_reset");
      }
      if (pending.end_stamp_ns < 0 || pending.end_stamp_ns <= lidar_only_input_stamp_ns_) {
        ++dropped_cloud_count_;
        return;
      }
      lidar_only_input_stamp_ns_ = pending.end_stamp_ns;
    }
    processCloud(pending);
    return;
  }
  if (!pointCloudTimeBounds(
      *message, point_time_field_, point_time_mode_, max_scan_duration_s_,
      point_time_header_tolerance_s_, require_point_time_,
      pending.start_stamp_ns, pending.end_stamp_ns, pending.has_point_time))
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++dropped_cloud_count_;
    }
    publishDiagnostic(
      rclcpp::Time(message->header.stamp), diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "degraded", "invalid_point_time_bounds");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_clouds_.push_back(std::move(pending));
    while (static_cast<int>(pending_clouds_.size()) > pending_cloud_limit_) {
      pending_clouds_.pop_front();
      ++dropped_cloud_count_;
    }
  }
}

void LioOdometry::processPendingClouds()
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (processing_cloud_) {
      return;
    }
    processing_cloud_ = true;
  }
  while (rclcpp::ok()) {
    std::optional<PendingCloud> pending;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!corrected_estimator_.initialized() || pending_clouds_.empty() ||
        latest_imu_stamp_ns_ < pending_clouds_.front().end_stamp_ns)
      {
        processing_cloud_ = false;
        processing_cloud_end_stamp_ns_ = 0;
        return;
      }
      pending = std::move(pending_clouds_.front());
      pending_clouds_.pop_front();
      processing_cloud_end_stamp_ns_ = pending->end_stamp_ns;
    }
    processCloud(*pending);
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  processing_cloud_ = false;
  processing_cloud_end_stamp_ns_ = 0;
}

void LioOdometry::processCloud(const PendingCloud & pending)
{
  if (pending.message->header.frame_id.empty()) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++dropped_cloud_count_;
    }
    publishDiagnostic(
      rclcpp::Time(pending.message->header.stamp),
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded", "cloud_frame_empty");
    return;
  }
  Eigen::Isometry3d base_T_sensor = Eigen::Isometry3d::Identity();
  if (pending.message->header.frame_id != base_frame_) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_, pending.message->header.frame_id,
        pending.message->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
      base_T_sensor = transformToIsometry(transform.transform);
    } catch (const tf2::TransformException & error) {
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++dropped_cloud_count_;
      }
      RCLCPP_WARN(get_logger(), "LIO cloud TF lookup failed: %s", error.what());
      publishDiagnostic(
        rclcpp::Time(pending.message->header.stamp),
        diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded", "cloud_tf_unavailable");
      return;
    }
  }

  if (!imu_enabled_) {
    processLidarOnlyCloud(pending, base_T_sensor);
    return;
  }

  LioStateEstimator estimator_at_end(state_config_);
  std::vector<TimedPose> trajectory;
  std::uint64_t snapshot_revision = 0U;
  bool first_lidar_reference = false;
  CloudT::ConstPtr submap_snapshot;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!corrected_estimator_.initialized() ||
      pending.end_stamp_ns <= last_processed_cloud_stamp_ns_)
    {
      ++dropped_cloud_count_;
      return;
    }
    estimator_at_end = corrected_estimator_;
    if (!buildImuTrajectory(
        pending.start_stamp_ns, pending.end_stamp_ns, estimator_at_end, trajectory))
    {
      ++dropped_cloud_count_;
      last_processed_cloud_stamp_ns_ = pending.end_stamp_ns;
      trajectory.clear();
    } else {
      last_processed_cloud_stamp_ns_ = pending.end_stamp_ns;
      snapshot_revision = state_revision_;
      first_lidar_reference = !have_lidar_reference_;
      std::lock_guard<std::mutex> submap_lock(submap_mutex_);
      submap_snapshot = rolling_submap_;
    }
  }
  if (trajectory.empty()) {
    publishDiagnostic(
      rclcpp::Time(pending.end_stamp_ns, RCL_ROS_TIME),
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded",
      "imu_coverage_unavailable");
    return;
  }

  std::size_t valid_input_points = 0U;
  const auto scan = prepareAndDeskewCloud(
    *pending.message, base_T_sensor, trajectory, odomTBase(estimator_at_end),
    pending.start_stamp_ns, pending.end_stamp_ns, pending.has_point_time,
    valid_input_points);
  const rclcpp::Time scan_stamp(pending.end_stamp_ns, RCL_ROS_TIME);
  const auto commit_prediction_only =
    [this, &estimator_at_end, &pending, snapshot_revision](
    const bool registration_rejected, LioStateEstimator & anchor_state,
    LioStateEstimator & live_state, std::uint64_t & publication_revision,
    std::uint64_t & committed_state_revision) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (snapshot_revision != state_revision_) {
        ++dropped_cloud_count_;
        return false;
      }
      if (registration_rejected) {
        ++rejected_registration_count_;
      } else {
        ++dropped_cloud_count_;
      }
      advanceCorrectedAnchor(estimator_at_end, pending.end_stamp_ns);
      committed_state_revision = state_revision_;
      anchor_state = corrected_estimator_;
      live_state = live_estimator_;
      publication_revision = publication_revision_;
      return true;
    };

  if (!scan || scan->size() < static_cast<std::size_t>(min_scan_points_)) {
    LioStateEstimator anchor_state(state_config_);
    LioStateEstimator live_state(state_config_);
    std::uint64_t publication_revision = 0U;
    std::uint64_t committed_state_revision = 0U;
    const bool committed = commit_prediction_only(
      false, anchor_state, live_state, publication_revision,
      committed_state_revision);
    if (committed) {
      publishOdometry(
        live_state, rclcpp::Time(live_state.stampNs(), RCL_ROS_TIME),
        publication_revision);
    }
    publishDiagnostic(
      scan_stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded",
      committed ? "insufficient_scan_points" : "state_changed_during_scan_processing",
      nullptr, scan ? scan->size() : 0U, submap_snapshot->size());
    return;
  }

  if (first_lidar_reference) {
    LioStateEstimator anchor_state(state_config_);
    LioStateEstimator live_state(state_config_);
    std::size_t map_points = 0U;
    std::uint64_t publication_revision = 0U;
    std::uint64_t committed_state_revision = 0U;
    bool committed = false;
    bool seeded = false;
    const bool seed_requested =
      scan->size() >= static_cast<std::size_t>(min_submap_points_);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (snapshot_revision == state_revision_ && !have_lidar_reference_) {
        advanceCorrectedAnchor(estimator_at_end, pending.end_stamp_ns);
        committed = true;
        committed_state_revision = state_revision_;
        if (!seed_requested) {
          ++dropped_cloud_count_;
        }
        anchor_state = corrected_estimator_;
        live_state = live_estimator_;
        publication_revision = publication_revision_;
      } else {
        ++dropped_cloud_count_;
      }
    }
    if (committed && seed_requested) {
      seeded = updateKeyframeMapIfCurrent(
        *scan, odomTBase(anchor_state), committed_state_revision, true, true,
        map_points);
      if (!seeded) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++dropped_cloud_count_;
        committed = false;
      }
    }
    if (committed) {
      const bool outputs_published = publishScanOutputsIfCurrent(
        anchor_state, *scan, scan_stamp, committed_state_revision);
      if (!outputs_published) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++dropped_cloud_count_;
        committed = false;
        seeded = false;
      }
      if (committed) {
        publishOdometry(
          live_state, rclcpp::Time(live_state.stampNs(), RCL_ROS_TIME),
          publication_revision);
      }
    }
    publishDiagnostic(
      scan_stamp,
      seeded ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      seeded ? "tracking" : "initializing",
      seeded ? "first_lidar_reference" :
      (committed ? "insufficient_first_submap_points" :
      "state_changed_during_scan_processing"),
      nullptr, scan->size(), map_points);
    return;
  }

  const auto local_map = buildLocalSubmap(
    submap_snapshot, odomTBase(estimator_at_end).translation());
  if (!local_map || local_map->size() < static_cast<std::size_t>(min_submap_points_)) {
    LioStateEstimator anchor_state(state_config_);
    LioStateEstimator live_state(state_config_);
    std::uint64_t publication_revision = 0U;
    std::uint64_t committed_state_revision = 0U;
    const bool committed = commit_prediction_only(
      false, anchor_state, live_state, publication_revision,
      committed_state_revision);
    if (committed) {
      publishOdometry(
        live_state, rclcpp::Time(live_state.stampNs(), RCL_ROS_TIME),
        publication_revision);
    }
    publishDiagnostic(
      scan_stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR, "lost",
      committed ? "insufficient_local_submap" : "state_changed_during_scan_processing",
      nullptr, scan->size(),
      local_map ? local_map->size() : 0U);
    return;
  }
  RegistrationOutcome registration = registerScan(
    *scan, *local_map, odomTBase(estimator_at_end));
  if (!registration.accepted) {
    LioStateEstimator anchor_state(state_config_);
    LioStateEstimator live_state(state_config_);
    std::uint64_t publication_revision = 0U;
    std::uint64_t committed_state_revision = 0U;
    bool committed = commit_prediction_only(
      true, anchor_state, live_state, publication_revision,
      committed_state_revision);
    if (committed) {
      const bool outputs_published = publishScanOutputsIfCurrent(
        anchor_state, *scan, scan_stamp, committed_state_revision);
      if (!outputs_published) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++dropped_cloud_count_;
        committed = false;
      }
      if (committed) {
        publishOdometry(
          live_state, rclcpp::Time(live_state.stampNs(), RCL_ROS_TIME),
          publication_revision);
      }
    }
    publishDiagnostic(
      scan_stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR, "lost",
      committed ? registration.reason : "state_changed_during_scan_processing",
      &registration, scan->size(), local_map->size());
    return;
  }

  constexpr double kDegreesToRadians = 0.017453292519943295769;
  const double translation_std = std::max(
    lidar_translation_std_min_m_, lidar_noise_scale_ * registration.rmse_m);
  const double rotation_std = std::max(
    lidar_rotation_std_min_deg_ * kDegreesToRadians,
    std::atan2(translation_std, std::max(5.0, local_submap_radius_m_ * 0.5)));
  Eigen::Matrix<double, 6, 6> base_measurement_covariance =
    Eigen::Matrix<double, 6, 6>::Zero();
  const double weak_variance_scale =
    lidar_weak_axis_noise_multiplier_ * lidar_weak_axis_noise_multiplier_ - 1.0;
  Eigen::Matrix3d translation_covariance = Eigen::Matrix3d::Identity() *
    translation_std * translation_std;
  Eigen::Matrix3d rotation_covariance_odom = Eigen::Matrix3d::Identity() *
    rotation_std * rotation_std;
  if (registration.observable_rank > 0 && registration.observable_rank < 6) {
    translation_covariance += translation_std * translation_std * weak_variance_scale *
      registration.weak_translation_axes.cwiseMax(0.0).cwiseMin(1.0).asDiagonal();
    rotation_covariance_odom += rotation_std * rotation_std * weak_variance_scale *
      registration.weak_rotation_axes.cwiseMax(0.0).cwiseMin(1.0).asDiagonal();
  }
  const Eigen::Matrix3d body_R_odom = registration.odom_T_base.linear().transpose();
  base_measurement_covariance.block<3, 3>(0, 0) = translation_covariance;
  base_measurement_covariance.block<3, 3>(3, 3) =
    body_R_odom * rotation_covariance_odom * body_R_odom.transpose();
  Eigen::Matrix<double, 6, 6> base_to_imu_measurement_jacobian =
    Eigen::Matrix<double, 6, 6>::Identity();
  base_to_imu_measurement_jacobian.block<3, 3>(0, 3) =
    -registration.odom_T_base.linear() * skewMatrix(base_p_imu_);
  const Eigen::Matrix<double, 6, 6> measurement_covariance =
    base_to_imu_measurement_jacobian * base_measurement_covariance *
    base_to_imu_measurement_jacobian.transpose();
  if (!estimator_at_end.correctBodyPose(
      basePoseToBodyPose(registration.odom_T_base), measurement_covariance,
      lidar_pose_nis_gate_))
  {
    registration.accepted = false;
    registration.reason = "eskf_pose_update_failed";
    LioStateEstimator anchor_state(state_config_);
    LioStateEstimator live_state(state_config_);
    std::uint64_t publication_revision = 0U;
    std::uint64_t committed_state_revision = 0U;
    bool committed = commit_prediction_only(
      true, anchor_state, live_state, publication_revision,
      committed_state_revision);
    if (committed) {
      const bool outputs_published = publishScanOutputsIfCurrent(
        anchor_state, *scan, scan_stamp, committed_state_revision);
      if (!outputs_published) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        ++dropped_cloud_count_;
        committed = false;
      }
      if (committed) {
        publishOdometry(
          live_state, rclcpp::Time(live_state.stampNs(), RCL_ROS_TIME),
          publication_revision);
      }
    }
    publishDiagnostic(
      scan_stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR, "lost",
      committed ? registration.reason : "state_changed_during_scan_processing",
      &registration, scan->size(), local_map->size());
    return;
  }

  LioStateEstimator anchor_state(state_config_);
  LioStateEstimator live_state(state_config_);
  bool committed = false;
  std::size_t map_points = local_map->size();
  std::uint64_t publication_revision = 0U;
  std::uint64_t committed_state_revision = 0U;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (snapshot_revision == state_revision_) {
      advanceCorrectedAnchor(estimator_at_end, pending.end_stamp_ns);
      ++accepted_registration_count_;
      committed_state_revision = state_revision_;
      anchor_state = corrected_estimator_;
      live_state = live_estimator_;
      publication_revision = publication_revision_;
      committed = true;
    } else {
      ++dropped_cloud_count_;
    }
  }
  if (committed && !updateKeyframeMapIfCurrent(
      *scan, odomTBase(anchor_state), committed_state_revision, false, false,
      map_points))
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ++dropped_cloud_count_;
    committed = false;
  }
  if (committed) {
    const bool outputs_published = publishScanOutputsIfCurrent(
      anchor_state, *scan, scan_stamp, committed_state_revision);
    if (!outputs_published) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++dropped_cloud_count_;
      committed = false;
    }
    if (committed) {
      publishOdometry(
        live_state, rclcpp::Time(live_state.stampNs(), RCL_ROS_TIME),
        publication_revision);
    }
  }
  publishDiagnostic(
    scan_stamp,
    committed ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    committed ? "tracking" : "degraded",
    committed ? "accepted" : "state_changed_during_scan_processing",
    &registration, scan->size(), map_points);
}

void LioOdometry::wheelCallback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (!wheel_correction_enabled_) {
    return;
  }
  if (message->header.frame_id != odom_frame_ || message->child_frame_id != base_frame_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring wheel odometry frames '%s' -> '%s'; expected '%s' -> '%s'",
      message->header.frame_id.c_str(), message->child_frame_id.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
    return;
  }
  const double speed = message->twist.twist.linear.x;
  const std::int64_t stamp_ns = rosStampNs(message->header.stamp);
  const double variance = message->twist.covariance[0];
  const double standard_deviation = std::isfinite(variance) && variance > 0.0 ?
    std::sqrt(variance) : wheel_fallback_std_mps_;
  std::optional<LioStateEstimator> publish_state;
  std::uint64_t publish_revision = 0U;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!corrected_estimator_.initialized() || !std::isfinite(speed) ||
      std::abs(speed) > wheel_max_abs_speed_mps_ ||
      stamp_ns <= corrected_estimator_.stampNs() ||
      (!wheel_observations_.empty() && stamp_ns <= wheel_observations_.back().stamp_ns) ||
      latest_imu_stamp_ns_ - stamp_ns < 0 ||
      static_cast<double>(latest_imu_stamp_ns_ - stamp_ns) * 1.0e-9 > wheel_max_age_s_)
    {
      return;
    }
    LioStateEstimator estimator_at_wheel = corrected_estimator_;
    std::vector<TimedPose> ignored_trajectory;
    if (!buildImuTrajectory(
        corrected_estimator_.stampNs(), stamp_ns, estimator_at_wheel, ignored_trajectory))
    {
      return;
    }
    const Eigen::Vector3d angular_velocity =
      estimator_at_wheel.lastMeasurement().angular_velocity_radps -
      estimator_at_wheel.gyroscopeBias();
    const Eigen::Vector3d lever_arm_velocity = angular_velocity.cross(base_p_imu_);
    const double predicted_imu_origin_speed =
      (estimator_at_wheel.orientation().conjugate() * estimator_at_wheel.velocity()).x();
    const double predicted_speed = predicted_imu_origin_speed - lever_arm_velocity.x();
    if (std::abs(speed - predicted_speed) > wheel_max_innovation_mps_) {
      return;
    }
    const WheelObservation observation{stamp_ns, speed, standard_deviation};
    if (!applyWheelObservation(estimator_at_wheel, observation)) {
      return;
    }
    wheel_observations_.push_back(observation);
    if (processing_cloud_ && stamp_ns <= processing_cloud_end_stamp_ns_) {
      ++state_revision_;
    }
    ++publication_revision_;
    invalidateOlderPublications(publication_revision_);
    repropagateLiveState();
    ++wheel_update_count_;
    publish_state = live_estimator_;
    publish_revision = publication_revision_;
  }
  if (publish_state.has_value()) {
    publishOdometry(
      *publish_state, rclcpp::Time(publish_state->stampNs(), RCL_ROS_TIME),
      publish_revision);
  }
}

bool LioOdometry::buildImuTrajectory(
  const std::int64_t start_stamp_ns, const std::int64_t end_stamp_ns,
  LioStateEstimator & estimator_at_end, std::vector<TimedPose> & trajectory) const
{
  trajectory.clear();
  if (!corrected_estimator_.initialized() || end_stamp_ns < start_stamp_ns ||
    end_stamp_ns > latest_imu_stamp_ns_ ||
    start_stamp_ns < corrected_estimator_.stampNs())
  {
    return false;
  }
  LioStateEstimator estimator = corrected_estimator_;
  auto propagate_exactly_to = [&estimator](const std::int64_t target_stamp_ns) {
      if (target_stamp_ns == estimator.stampNs()) {
        return true;
      }
      return estimator.propagateTo(target_stamp_ns, estimator.lastMeasurement()) ==
             ImuPropagationStatus::kApplied;
    };
  std::size_t sample_index = 0U;
  while (sample_index < imu_buffer_.size() &&
    imu_buffer_[sample_index].stamp_ns <= estimator.stampNs())
  {
    ++sample_index;
  }
  std::size_t wheel_index = 0U;
  while (wheel_index < wheel_observations_.size() &&
    wheel_observations_[wheel_index].stamp_ns <= estimator.stampNs())
  {
    ++wheel_index;
  }
  auto record_pose = [&trajectory, this, &estimator, start_stamp_ns]() {
      if (estimator.stampNs() < start_stamp_ns) {
        return;
      }
      const TimedPose pose{estimator.stampNs(), odomTBase(estimator)};
      if (!trajectory.empty() && trajectory.back().stamp_ns == pose.stamp_ns) {
        trajectory.back() = pose;
      } else {
        trajectory.push_back(pose);
      }
    };
  auto process_until =
    [this, &estimator, &sample_index, &wheel_index, &record_pose](
    const std::int64_t limit_stamp_ns) {
      while (true) {
        const std::int64_t next_imu_stamp = sample_index < imu_buffer_.size() ?
          imu_buffer_[sample_index].stamp_ns : std::numeric_limits<std::int64_t>::max();
        const std::int64_t next_wheel_stamp = wheel_index < wheel_observations_.size() ?
          wheel_observations_[wheel_index].stamp_ns :
          std::numeric_limits<std::int64_t>::max();
        if (std::min(next_imu_stamp, next_wheel_stamp) > limit_stamp_ns) {
          return true;
        }
        if (next_imu_stamp <= next_wheel_stamp) {
          if (estimator.propagate(imu_buffer_[sample_index]) !=
            ImuPropagationStatus::kApplied)
          {
            return false;
          }
          ++sample_index;
        } else {
          if (next_wheel_stamp > estimator.stampNs() &&
            estimator.propagateTo(next_wheel_stamp, estimator.lastMeasurement()) !=
            ImuPropagationStatus::kApplied)
          {
            return false;
          }
          if (!applyWheelObservation(estimator, wheel_observations_[wheel_index])) {
            return false;
          }
          ++wheel_index;
        }
        record_pose();
      }
    };
  if (!process_until(start_stamp_ns) || !propagate_exactly_to(start_stamp_ns)) {
    return false;
  }
  record_pose();
  if (!process_until(end_stamp_ns) || !propagate_exactly_to(end_stamp_ns)) {
    return false;
  }
  record_pose();
  if (trajectory.empty() || trajectory.front().stamp_ns != start_stamp_ns ||
    trajectory.back().stamp_ns != end_stamp_ns)
  {
    return false;
  }
  estimator_at_end = estimator;
  return true;
}

bool LioOdometry::applyWheelObservation(
  LioStateEstimator & estimator, const WheelObservation & observation) const
{
  if (!estimator.initialized() || estimator.stampNs() != observation.stamp_ns ||
    !std::isfinite(observation.base_forward_speed_mps) ||
    !std::isfinite(observation.standard_deviation_mps) ||
    observation.standard_deviation_mps <= 0.0)
  {
    return false;
  }
  const Eigen::Vector3d angular_velocity =
    estimator.lastMeasurement().angular_velocity_radps - estimator.gyroscopeBias();
  const Eigen::Vector3d lever_arm_velocity = angular_velocity.cross(base_p_imu_);
  const double equivalent_imu_origin_speed =
    observation.base_forward_speed_mps + lever_arm_velocity.x();
  return estimator.correctBodyForwardVelocity(
    equivalent_imu_origin_speed, observation.standard_deviation_mps);
}

void LioOdometry::processLidarOnlyCloud(
  const PendingCloud & pending, const Eigen::Isometry3d & base_T_sensor)
{
  const rclcpp::Time stamp(pending.end_stamp_ns, RCL_ROS_TIME);
  std::size_t valid_input_points = 0U;
  const auto scan = prepareAndDeskewCloud(
    *pending.message, base_T_sensor, {}, Eigen::Isometry3d::Identity(),
    pending.start_stamp_ns, pending.end_stamp_ns, false, valid_input_points);
  if (!scan || scan->size() < static_cast<std::size_t>(min_scan_points_)) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++dropped_cloud_count_;
    }
    publishDiagnostic(stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "degraded", "insufficient_scan_points", nullptr, scan ? scan->size() : 0U);
    return;
  }

  Eigen::Isometry3d previous_pose;
  std::int64_t previous_stamp_ns;
  std::uint64_t revision;
  bool first_reference;
  CloudT::ConstPtr submap;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    previous_pose = lidar_only_pose_;
    previous_stamp_ns = lidar_only_pose_stamp_ns_;
    first_reference = !have_lidar_reference_;
    revision = state_revision_;
    std::lock_guard<std::mutex> submap_lock(submap_mutex_);
    submap = rolling_submap_;
  }
  RegistrationOutcome registration;
  registration.odom_T_base = previous_pose;
  std::size_t map_points = submap->size();
  if (first_reference) {
    registration.accepted = scan->size() >= static_cast<std::size_t>(min_submap_points_);
    registration.reason = registration.accepted ?
      "first_lidar_reference" : "insufficient_first_submap_points";
  } else {
    const auto local_map = buildLocalSubmap(submap, previous_pose.translation());
    map_points = local_map->size();
    if (map_points < static_cast<std::size_t>(min_submap_points_)) {
      registration.reason = "insufficient_local_submap";
    } else {
      // No wheel, inertial integration, gravity alignment or ESKF correction.
      registration = registerScan(*scan, *local_map, previous_pose);
    }
  }
  if (!registration.accepted) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      ++rejected_registration_count_;
    }
    // Without a motion source there is no valid new odometry on rejection.
    // Keep the accepted pose/submap and do not publish a stale pose as fresh.
    publishDiagnostic(stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "lost", registration.reason, &registration, scan->size(), map_points);
    return;
  }
  const auto & pose = registration.odom_T_base;
  if (!updateKeyframeMapIfCurrent(
      *scan, pose, revision, first_reference, true, map_points))
  {
    return;
  }

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = odom_frame_;
  odometry.child_frame_id = base_frame_;
  odometry.pose.pose = isometryToPose(pose);
  // Deliberately uncalibrated conservative covariance; not an ESKF posterior.
  for (std::size_t axis = 0; axis < 6U; ++axis) {
    odometry.pose.covariance[axis * 7U] = 1.0e3;
    odometry.twist.covariance[axis * 7U] = 1.0e3;
  }
  if (previous_stamp_ns >= 0 && pending.end_stamp_ns > previous_stamp_ns) {
    const double dt = static_cast<double>(pending.end_stamp_ns - previous_stamp_ns) * 1.0e-9;
    const Eigen::Vector3d velocity = pose.linear().transpose() *
      (pose.translation() - previous_pose.translation()) / dt;
    const Eigen::AngleAxisd delta_rotation(previous_pose.linear().transpose() * pose.linear());
    const Eigen::Vector3d angular_velocity = delta_rotation.axis() * delta_rotation.angle() / dt;
    odometry.twist.twist.linear.x = velocity.x();
    odometry.twist.twist.linear.y = velocity.y();
    odometry.twist.twist.linear.z = velocity.z();
    odometry.twist.twist.angular.x = angular_velocity.x();
    odometry.twist.twist.angular.y = angular_velocity.y();
    odometry.twist.twist.angular.z = angular_velocity.z();
  }
  sensor_msgs::msg::PointCloud2 cloud_message;
  pcl::toROSMsg(*scan, cloud_message);
  cloud_message.header = odometry.header;
  cloud_message.header.frame_id = base_frame_;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (revision != state_revision_) {
      return;
    }
    lidar_only_pose_ = pose;
    lidar_only_pose_stamp_ns_ = pending.end_stamp_ns;
    last_processed_cloud_stamp_ns_ = pending.end_stamp_ns;
    last_propagation_status_ = "disabled_lidar_only";
    if (!first_reference) {
      ++accepted_registration_count_;
    }
    // Existing topic names are retained, including the legacy deskewed name.
    scan_odom_pub_->publish(odometry);
    deskewed_cloud_pub_->publish(cloud_message);
    odom_pub_->publish(odometry);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform = isometryToTransform(pose);
      tf_broadcaster_->sendTransform(transform);
    }
  }
  publishDiagnostic(stamp, diagnostic_msgs::msg::DiagnosticStatus::OK,
    "tracking", registration.reason, first_reference ? nullptr : &registration,
    scan->size(), map_points);
}

LioOdometry::CloudT::Ptr LioOdometry::prepareAndDeskewCloud(
  const sensor_msgs::msg::PointCloud2 & message,
  const Eigen::Isometry3d & base_T_sensor,
  const std::vector<TimedPose> & trajectory,
  const Eigen::Isometry3d & odom_T_base_reference,
  const std::int64_t start_stamp_ns, const std::int64_t end_stamp_ns,
  const bool has_point_time, std::size_t & valid_input_points) const
{
  CloudT::Ptr deskewed(new CloudT);
  valid_input_points = 0U;
  if (message.is_bigendian || message.point_step == 0U ||
    (imu_enabled_ && trajectory.empty()) ||
    message.row_step < static_cast<std::uint64_t>(message.width) * message.point_step ||
    static_cast<std::uint64_t>(message.row_step) * message.height > message.data.size())
  {
    return deskewed;
  }
  const PointField * x_field = findField(message, "x");
  const PointField * y_field = findField(message, "y");
  const PointField * z_field = findField(message, "z");
  const PointField * time_field = imu_enabled_ && has_point_time ?
    findField(message, point_time_field_) : nullptr;
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr ||
    (imu_enabled_ && has_point_time && time_field == nullptr))
  {
    return deskewed;
  }
  for (const PointField * field : {x_field, y_field, z_field, time_field}) {
    if (field != nullptr &&
      (datatypeSize(field->datatype) == 0U ||
      field->offset + datatypeSize(field->datatype) > message.point_step))
    {
      return deskewed;
    }
  }

  deskewed->reserve(static_cast<std::size_t>(message.width) * message.height);
  const Eigen::Isometry3d reference_T_odom = odom_T_base_reference.inverse();
  const std::int64_t header_stamp_ns = rosStampNs(message.header.stamp);
  for (std::uint32_t row = 0; row < message.height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * message.row_step;
    for (std::uint32_t column = 0; column < message.width; ++column) {
      const std::size_t offset = row_offset + static_cast<std::size_t>(column) * message.point_step;
      if (offset + message.point_step > message.data.size()) {
        return CloudT::Ptr(new CloudT);
      }
      const std::uint8_t * point_data = message.data.data() + offset;
      const double x = readNumericField(point_data, *x_field);
      const double y = readNumericField(point_data, *y_field);
      const double z = readNumericField(point_data, *z_field);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      ++valid_input_points;
      std::int64_t point_stamp_ns = end_stamp_ns;
      if (time_field != nullptr) {
        point_stamp_ns = pointStampNs(
          readNumericField(point_data, *time_field), point_time_mode_, header_stamp_ns);
        if (point_stamp_ns < start_stamp_ns || point_stamp_ns > end_stamp_ns) {
          continue;
        }
      }
      const Eigen::Vector3d point_sensor(x, y, z);
      const Eigen::Vector3d point_base = base_T_sensor * point_sensor;
      const double range = std::hypot(point_base.x(), point_base.y());
      if (range < min_range_m_ || range > max_range_m_ ||
        point_base.z() < z_min_m_ || point_base.z() > z_max_m_)
      {
        continue;
      }
      if (crop_box_enabled_ &&
        (point_base.array() >= crop_min_.array()).all() &&
        (point_base.array() <= crop_max_.array()).all())
      {
        continue;
      }
      Eigen::Vector3d point_reference = point_base;
      if (imu_enabled_) {
        const Eigen::Isometry3d odom_T_base_point = interpolatePose(trajectory, point_stamp_ns);
        point_reference = reference_T_odom * odom_T_base_point * point_base;
      }
      deskewed->push_back(
        PointT(
          static_cast<float>(point_reference.x()),
          static_cast<float>(point_reference.y()),
          static_cast<float>(point_reference.z())));
    }
  }
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(scan_voxel_leaf_m_, scan_voxel_leaf_m_, scan_voxel_leaf_m_);
  voxel.setInputCloud(deskewed);
  CloudT::Ptr filtered(new CloudT);
  voxel.filter(*filtered);
  return filtered;
}

LioOdometry::RegistrationOutcome LioOdometry::registerScan(
  const CloudT & scan_base, const CloudT & local_map,
  const Eigen::Isometry3d & prediction) const
{
  RegistrationOutcome outcome;
  outcome.odom_T_base = prediction;
  const auto start = std::chrono::steady_clock::now();
  if (registration_method_ == "voxel_surfel") {
    const ::gole::slam_3d_core::VoxelSurfelRegistration registration(surfel_config_);
    const auto result = registration.align(scan_base, local_map, prediction);
    outcome.converged = result.converged;
    outcome.accepted = result.accepted;
    outcome.reason = result.reason;
    outcome.odom_T_base = result.target_T_source;
    outcome.fitness = std::isfinite(result.rmse) ? result.rmse * result.rmse :
      std::numeric_limits<double>::infinity();
    outcome.rmse_m = result.rmse;
    outcome.p95_m = result.p95_abs_residual;
    outcome.correspondence_ratio = result.correspondence_ratio;
    outcome.observable_rank = result.observable_rank;
    outcome.weak_rotation_axes = result.weak_rotation_axes;
    outcome.weak_translation_axes = result.weak_translation_axes;
    outcome.runtime_ms = result.total_time_ms;
  } else {
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> registration;
    registration.setInputSource(scan_base.makeShared());
    registration.setInputTarget(local_map.makeShared());
    registration.setMaximumIterations(gicp_max_iterations_);
    registration.setMaxCorrespondenceDistance(gicp_max_correspondence_m_);
    registration.setTransformationEpsilon(1.0e-3);
    registration.setEuclideanFitnessEpsilon(1.0e-3);
    CloudT aligned;
    registration.align(aligned, prediction.matrix().cast<float>());
    outcome.converged = registration.hasConverged();
    outcome.accepted = outcome.converged;
    outcome.reason = outcome.converged ? "converged" : "not_converged";
    outcome.odom_T_base.matrix() = registration.getFinalTransformation().cast<double>();
    outcome.fitness = outcome.converged ?
      registration.getFitnessScore(gicp_max_correspondence_m_) :
      std::numeric_limits<double>::infinity();
    outcome.rmse_m = std::sqrt(std::max(0.0, outcome.fitness));
    outcome.runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  }
  if (!outcome.accepted) {
    return outcome;
  }
  if (!outcome.odom_T_base.matrix().allFinite() || !std::isfinite(outcome.fitness)) {
    outcome.accepted = false;
    outcome.reason = "invalid_result";
    return outcome;
  }
  if (outcome.fitness > fitness_threshold_) {
    outcome.accepted = false;
    outcome.reason = "fitness";
    return outcome;
  }
  const Eigen::Isometry3d correction = prediction.inverse() * outcome.odom_T_base;
  const double correction_translation = correction.translation().norm();
  const double correction_rotation_deg = rotationAngle(correction.linear()) *
    57.295779513082320876;
  if (!std::isfinite(correction_translation) || !std::isfinite(correction_rotation_deg) ||
    correction_translation > max_correction_translation_m_)
  {
    outcome.accepted = false;
    outcome.reason = "translation_correction";
  } else if (correction_rotation_deg > max_correction_rotation_deg_) {
    outcome.accepted = false;
    outcome.reason = "rotation_correction";
  }
  return outcome;
}

LioOdometry::CloudT::Ptr LioOdometry::buildLocalSubmap(
  const CloudT::ConstPtr & submap_snapshot, const Eigen::Vector3d & center) const
{
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(submap_snapshot);
  crop.setMin(
    Eigen::Vector4f(
      center.x() - local_submap_radius_m_, center.y() - local_submap_radius_m_,
      center.z() - local_submap_radius_m_, 1.0));
  crop.setMax(
    Eigen::Vector4f(
      center.x() + local_submap_radius_m_, center.y() + local_submap_radius_m_,
      center.z() + local_submap_radius_m_, 1.0));
  CloudT::Ptr result(new CloudT);
  crop.filter(*result);
  return result;
}

bool LioOdometry::updateKeyframeMapIfCurrent(
  const CloudT & scan_base, const Eigen::Isometry3d & odom_T_base,
  const std::uint64_t expected_state_revision, const bool force_keyframe,
  const bool mark_lidar_reference, std::size_t & map_points)
{
  std::deque<LidarKeyframe> candidate_keyframes;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (state_revision_ != expected_state_revision) {
      return false;
    }
    std::lock_guard<std::mutex> submap_lock(submap_mutex_);
    if (!force_keyframe && !shouldAddKeyframe(keyframes_, odom_T_base)) {
      map_points = rolling_submap_->size();
      if (mark_lidar_reference) {
        have_lidar_reference_ = true;
      }
      return true;
    }
    candidate_keyframes = keyframes_;
  }

  LidarKeyframe keyframe;
  keyframe.odom_T_base = odom_T_base;
  pcl::transformPointCloud(
    scan_base, *keyframe.cloud_odom, odom_T_base.matrix().cast<float>());
  candidate_keyframes.push_back(std::move(keyframe));
  while (static_cast<int>(candidate_keyframes.size()) > max_keyframes_) {
    candidate_keyframes.pop_front();
  }

  const CloudT::Ptr candidate_submap = buildRollingSubmap(candidate_keyframes);
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (state_revision_ != expected_state_revision) {
      return false;
    }
    std::lock_guard<std::mutex> submap_lock(submap_mutex_);
    keyframes_ = std::move(candidate_keyframes);
    rolling_submap_ = candidate_submap;
    map_points = rolling_submap_->size();
    if (mark_lidar_reference) {
      have_lidar_reference_ = true;
    }
  }
  return true;
}

LioOdometry::CloudT::Ptr LioOdometry::buildRollingSubmap(
  const std::deque<LidarKeyframe> & keyframes) const
{
  CloudT::Ptr merged(new CloudT);
  std::size_t point_count = 0U;
  for (const auto & keyframe : keyframes) {
    point_count += keyframe.cloud_odom->size();
  }
  merged->reserve(point_count);
  for (const auto & keyframe : keyframes) {
    *merged += *keyframe.cloud_odom;
  }
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(submap_voxel_leaf_m_, submap_voxel_leaf_m_, submap_voxel_leaf_m_);
  voxel.setInputCloud(merged);
  CloudT::Ptr result(new CloudT);
  voxel.filter(*result);
  return result;
}

bool LioOdometry::shouldAddKeyframe(
  const std::deque<LidarKeyframe> & keyframes,
  const Eigen::Isometry3d & pose) const
{
  if (keyframes.empty()) {
    return true;
  }
  const Eigen::Isometry3d delta = keyframes.back().odom_T_base.inverse() * pose;
  return delta.translation().norm() >= keyframe_translation_m_ ||
         rotationAngle(delta.linear()) * 57.295779513082320876 >= keyframe_rotation_deg_;
}

void LioOdometry::repropagateLiveState()
{
  if (latest_imu_stamp_ns_ <= corrected_estimator_.stampNs()) {
    live_estimator_ = corrected_estimator_;
    return;
  }
  LioStateEstimator propagated = corrected_estimator_;
  std::vector<TimedPose> ignored_trajectory;
  if (buildImuTrajectory(
      corrected_estimator_.stampNs(), latest_imu_stamp_ns_,
      propagated, ignored_trajectory))
  {
    live_estimator_ = propagated;
    last_propagation_status_ = "applied";
  } else {
    last_propagation_status_ = "repropagation_failed";
  }
}

void LioOdometry::advanceCorrectedAnchor(
  const LioStateEstimator & estimator_at_stamp, const std::int64_t stamp_ns)
{
  corrected_estimator_ = estimator_at_stamp;
  while (!imu_buffer_.empty() && imu_buffer_.front().stamp_ns <= stamp_ns) {
    imu_buffer_.pop_front();
  }
  while (!wheel_observations_.empty() && wheel_observations_.front().stamp_ns <= stamp_ns) {
    wheel_observations_.pop_front();
  }
  repropagateLiveState();
  ++state_revision_;
  ++publication_revision_;
  invalidateOlderPublications(publication_revision_);
}

void LioOdometry::invalidateOlderPublications(const std::uint64_t minimum_revision)
{
  std::lock_guard<std::mutex> lock(odom_publish_mutex_);
  minimum_valid_publication_revision_ = std::max(
    minimum_valid_publication_revision_, minimum_revision);
}

void LioOdometry::boundImuBuffer()
{
  if (!corrected_estimator_.initialized() || imu_buffer_.empty() || processing_cloud_) {
    return;
  }
  const std::int64_t cutoff_ns = latest_imu_stamp_ns_ - static_cast<std::int64_t>(
    std::llround(imu_buffer_duration_s_ * 1.0e9));
  std::int64_t target_stamp_ns = cutoff_ns;
  if (!pending_clouds_.empty()) {
    target_stamp_ns = std::min(target_stamp_ns, pending_clouds_.front().start_stamp_ns);
  }
  if (target_stamp_ns <= corrected_estimator_.stampNs()) {
    while (!imu_buffer_.empty() &&
      imu_buffer_.front().stamp_ns <= corrected_estimator_.stampNs())
    {
      imu_buffer_.pop_front();
    }
    return;
  }

  // Without LiDAR, cutoff advances at every IMU sample. Replaying the entire
  // retained window for each sample is quadratic, so move the anchor in
  // bounded batches while allowing at most 20% temporary buffer overrun.
  const std::int64_t minimum_advance_ns = static_cast<std::int64_t>(
    std::llround(std::max(max_scan_duration_s_, 0.2 * imu_buffer_duration_s_) * 1.0e9));
  if (target_stamp_ns - corrected_estimator_.stampNs() < minimum_advance_ns) {
    return;
  }

  LioStateEstimator bounded_anchor = corrected_estimator_;
  std::vector<TimedPose> ignored_trajectory;
  if (!buildImuTrajectory(
      corrected_estimator_.stampNs(), target_stamp_ns,
      bounded_anchor, ignored_trajectory))
  {
    return;
  }
  advanceCorrectedAnchor(bounded_anchor, target_stamp_ns);
}

Eigen::Isometry3d LioOdometry::odomTBase(const LioStateEstimator & estimator) const
{
  const Eigen::Isometry3d odom_T_body = estimator.odomTBody();
  Eigen::Isometry3d body_T_base = Eigen::Isometry3d::Identity();
  body_T_base.translation() = -base_p_imu_;
  return odom_T_body * body_T_base;
}

Eigen::Isometry3d LioOdometry::basePoseToBodyPose(
  const Eigen::Isometry3d & odom_T_base) const
{
  Eigen::Isometry3d base_T_body = Eigen::Isometry3d::Identity();
  base_T_body.translation() = base_p_imu_;
  return odom_T_base * base_T_body;
}

void LioOdometry::resetForTimeEpoch(const std::string & reason)
{
  corrected_estimator_.reset();
  live_estimator_.reset();
  imu_initializer_.reset();
  imu_buffer_.clear();
  wheel_observations_.clear();
  pending_clouds_.clear();
  {
    std::lock_guard<std::mutex> submap_lock(submap_mutex_);
    keyframes_.clear();
    rolling_submap_ = CloudT::Ptr(new CloudT);
  }
  have_lidar_reference_ = false;
  lidar_only_pose_.setIdentity();
  lidar_only_input_stamp_ns_ = -1;
  lidar_only_pose_stamp_ns_ = -1;
  last_processed_cloud_stamp_ns_ = 0;
  latest_imu_stamp_ns_ = 0;
  if (imu_enabled_) {
    ++imu_reset_count_;
  }
  ++state_revision_;
  ++publication_revision_;
  invalidateOlderPublications(publication_revision_);
  last_propagation_status_ = reason;
}

nav_msgs::msg::Odometry LioOdometry::makeOdometry(
  const LioStateEstimator & estimator, const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Odometry message;
  message.header.stamp = stamp;
  message.header.frame_id = odom_frame_;
  message.child_frame_id = base_frame_;
  message.pose.pose = isometryToPose(odomTBase(estimator));
  const auto & covariance = estimator.covariance();
  Eigen::Matrix<double, 6, LioStateEstimator::kStateDimension> pose_jacobian =
    Eigen::Matrix<double, 6, LioStateEstimator::kStateDimension>::Zero();
  pose_jacobian.block<3, 3>(0, 0).setIdentity();
  pose_jacobian.block<3, 3>(0, 6) =
    estimator.orientation().toRotationMatrix() * skewMatrix(base_p_imu_);
  pose_jacobian.block<3, 3>(3, 6) = estimator.orientation().toRotationMatrix();
  const Eigen::Matrix<double, 6, 6> base_pose_covariance =
    pose_jacobian * covariance * pose_jacobian.transpose();
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      message.pose.covariance[static_cast<std::size_t>(row * 6 + column)] =
        base_pose_covariance(row, column);
    }
  }
  const Eigen::Vector3d angular_velocity =
    estimator.lastMeasurement().angular_velocity_radps - estimator.gyroscopeBias();
  const Eigen::Vector3d velocity_base = estimator.orientation().conjugate() *
    estimator.velocity() - angular_velocity.cross(base_p_imu_);
  message.twist.twist.linear.x = velocity_base.x();
  message.twist.twist.linear.y = velocity_base.y();
  message.twist.twist.linear.z = velocity_base.z();
  message.twist.twist.angular.x = angular_velocity.x();
  message.twist.twist.angular.y = angular_velocity.y();
  message.twist.twist.angular.z = angular_velocity.z();
  Eigen::Matrix<double, 6, LioStateEstimator::kStateDimension> twist_jacobian =
    Eigen::Matrix<double, 6, LioStateEstimator::kStateDimension>::Zero();
  const Eigen::Matrix3d body_R_odom = estimator.orientation().conjugate().toRotationMatrix();
  const Eigen::Vector3d velocity_imu_body = body_R_odom * estimator.velocity();
  twist_jacobian.block<3, 3>(0, 3) = body_R_odom;
  twist_jacobian.block<3, 3>(0, 6) = skewMatrix(velocity_imu_body);
  twist_jacobian.block<3, 3>(0, 9) = -skewMatrix(base_p_imu_);
  twist_jacobian.block<3, 3>(3, 9) = -Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 6, 6> base_twist_covariance =
    twist_jacobian * covariance * twist_jacobian.transpose();
  Eigen::Matrix<double, 6, 3> gyro_noise_jacobian =
    Eigen::Matrix<double, 6, 3>::Zero();
  gyro_noise_jacobian.block<3, 3>(0, 0) = skewMatrix(base_p_imu_);
  gyro_noise_jacobian.block<3, 3>(3, 0).setIdentity();
  base_twist_covariance += gyro_noise_jacobian *
    (Eigen::Matrix3d::Identity() * state_config_.gyroscope_noise_radps_sqrt_hz *
    state_config_.gyroscope_noise_radps_sqrt_hz) * gyro_noise_jacobian.transpose();
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      message.twist.covariance[static_cast<std::size_t>(row * 6 + column)] =
        base_twist_covariance(row, column);
    }
  }
  return message;
}

void LioOdometry::publishOdometry(
  const LioStateEstimator & estimator, const rclcpp::Time & stamp,
  const std::uint64_t publication_revision)
{
  if (!estimator.initialized()) {
    return;
  }
  const std::int64_t stamp_ns = stamp.nanoseconds();
  std::lock_guard<std::mutex> lock(odom_publish_mutex_);
  if (publication_revision < minimum_valid_publication_revision_) {
    return;
  }
  if (have_published_odom_) {
    if (publication_revision < last_published_revision_) {
      return;
    }
    if (stamp_ns <= last_published_odom_stamp_ns_) {
      const std::int64_t epoch_reset_threshold_ns = static_cast<std::int64_t>(
        std::llround(imu_epoch_reset_threshold_s_ * 1.0e9));
      const bool newer_epoch = publication_revision > last_published_revision_ &&
        stamp_ns < last_published_odom_stamp_ns_ - epoch_reset_threshold_ns;
      if (!newer_epoch) {
        return;
      }
    }
  }
  const auto message = makeOdometry(estimator, stamp);
  odom_pub_->publish(message);
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = message.header;
    transform.child_frame_id = base_frame_;
    transform.transform = isometryToTransform(odomTBase(estimator));
    tf_broadcaster_->sendTransform(transform);
  }
  have_published_odom_ = true;
  last_published_odom_stamp_ns_ = stamp_ns;
  last_published_revision_ = publication_revision;
}

bool LioOdometry::publishScanOutputsIfCurrent(
  const LioStateEstimator & estimator, const CloudT & cloud,
  const rclcpp::Time & stamp, const std::uint64_t expected_state_revision)
{
  if (!estimator.initialized()) {
    return false;
  }
  const auto odometry_message = makeOdometry(estimator, stamp);
  sensor_msgs::msg::PointCloud2 cloud_message;
  pcl::toROSMsg(cloud, cloud_message);
  cloud_message.header.stamp = stamp;
  cloud_message.header.frame_id = base_frame_;

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_revision_ != expected_state_revision) {
    return false;
  }
  // This scan-time posterior/cloud pair is the exact localizer synchronization
  // channel. Holding the state gate across both publishes prevents an epoch
  // reset from splitting the pair or releasing outputs from an older revision.
  scan_odom_pub_->publish(odometry_message);
  deskewed_cloud_pub_->publish(cloud_message);
  return true;
}

void LioOdometry::publishDiagnostic(
  const rclcpp::Time & stamp, const int level, const std::string & state,
  const std::string & reason, const RegistrationOutcome * registration,
  const std::size_t scan_points, const std::size_t map_points)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  std::size_t keyframe_count = 0U;
  {
    std::lock_guard<std::mutex> submap_lock(submap_mutex_);
    keyframe_count = keyframes_.size();
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = static_cast<std::uint8_t>(level);
  status.name = get_fully_qualified_name() +
    std::string(imu_enabled_ ? ": lidar_inertial_odometry" : ": lidar_odometry");
  status.hardware_id = imu_enabled_ ? "lidar_imu" : "lidar_only";
  status.message = state + ": " + reason;
  status.values.push_back(keyValue("state", state));
  status.values.push_back(keyValue("reason", reason));
  status.values.push_back(keyValue("fusion_mode", imu_enabled_ ? "lidar_imu" : "lidar_only"));
  status.values.push_back(keyValue("imu_enabled", imu_enabled_ ? "true" : "false"));
  status.values.push_back(keyValue("imu_bias_estimation_enabled",
    imu_enabled_ && state_config_.imu_bias_estimation_enabled ? "true" : "false"));
  status.values.push_back(keyValue("imu_gyroscope_bias_norm_radps",
    std::to_string(live_estimator_.gyroscopeBias().norm())));
  status.values.push_back(keyValue("imu_accelerometer_bias_norm_mps2",
    std::to_string(live_estimator_.accelerometerBias().norm())));
  status.values.push_back(keyValue("deskew_enabled", imu_enabled_ ? "true" : "false"));
  status.values.push_back(keyValue("odom_publish_source", imu_enabled_ ? "imu" : "lidar"));
  status.values.push_back(keyValue("scan_stamp_reference", imu_enabled_ ? "scan_end" : "header"));
  status.values.push_back(
    keyValue(
      "wheel_correction_enabled", wheel_correction_enabled_ ? "true" : "false"));
  status.values.push_back(keyValue("wheel_update_count", std::to_string(wheel_update_count_)));
  status.values.push_back(keyValue("registration_method", registration_method_));
  status.values.push_back(keyValue("scan_points", std::to_string(scan_points)));
  status.values.push_back(keyValue("local_submap_points", std::to_string(map_points)));
  status.values.push_back(keyValue("keyframes", std::to_string(keyframe_count)));
  status.values.push_back(
    keyValue(
      "accepted_registrations", std::to_string(accepted_registration_count_)));
  status.values.push_back(
    keyValue(
      "rejected_registrations", std::to_string(rejected_registration_count_)));
  status.values.push_back(keyValue("dropped_clouds", std::to_string(dropped_cloud_count_)));
  status.values.push_back(keyValue("imu_resets", std::to_string(imu_reset_count_)));
  status.values.push_back(
    keyValue(
      "imu_gap_recoveries", std::to_string(imu_gap_recovery_count_)));
  status.values.push_back(keyValue("imu_propagation_status", last_propagation_status_));
  status.values.push_back(
    keyValue(
      "registration_reason", registration == nullptr ? "not_run" : registration->reason));
  status.values.push_back(
    keyValue(
      "registration_fitness", std::to_string(
        registration == nullptr ? 0.0 : registration->fitness)));
  status.values.push_back(
    keyValue(
      "registration_rmse_m", std::to_string(
        registration == nullptr ? 0.0 : registration->rmse_m)));
  status.values.push_back(
    keyValue(
      "registration_p95_m", std::to_string(
        registration == nullptr ? 0.0 : registration->p95_m)));
  status.values.push_back(
    keyValue(
      "registration_correspondence_ratio", std::to_string(
        registration == nullptr ? 0.0 : registration->correspondence_ratio)));
  status.values.push_back(
    keyValue(
      "registration_observable_rank", std::to_string(
        registration == nullptr ? 0 : registration->observable_rank)));
  status.values.push_back(
    keyValue(
      "registration_weak_rotation_xyz",
      registration == nullptr ? "0,0,0" :
      std::to_string(registration->weak_rotation_axes.x()) + "," +
      std::to_string(registration->weak_rotation_axes.y()) + "," +
      std::to_string(registration->weak_rotation_axes.z())));
  status.values.push_back(
    keyValue(
      "registration_weak_translation_xyz",
      registration == nullptr ? "0,0,0" :
      std::to_string(registration->weak_translation_axes.x()) + "," +
      std::to_string(registration->weak_translation_axes.y()) + "," +
      std::to_string(registration->weak_translation_axes.z())));
  status.values.push_back(
    keyValue(
      "registration_runtime_ms", std::to_string(
        registration == nullptr ? 0.0 : registration->runtime_ms)));
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);
}

bool LioOdometry::pointCloudTimeBounds(
  const sensor_msgs::msg::PointCloud2 & message, const std::string & field_name,
  const std::string & mode, const double max_scan_duration_s,
  const double header_tolerance_s, const bool require_point_time,
  std::int64_t & start_stamp_ns, std::int64_t & end_stamp_ns, bool & has_point_time)
{
  const std::int64_t header_stamp_ns = rosStampNs(message.header.stamp);
  start_stamp_ns = header_stamp_ns;
  end_stamp_ns = header_stamp_ns;
  has_point_time = false;
  if (message.is_bigendian || message.point_step == 0U || message.height == 0U ||
    message.width == 0U || header_stamp_ns < 0)
  {
    return false;
  }
  const PointField * field = findField(message, field_name);
  if (field == nullptr) {
    return !require_point_time;
  }
  if (field->count != 1U || datatypeSize(field->datatype) == 0U ||
    field->offset + datatypeSize(field->datatype) > message.point_step)
  {
    return false;
  }
  if (mode == "relative_nanoseconds" && field->datatype != PointField::UINT32) {
    return false;
  }
  if (mode == "absolute_seconds" && field->datatype != PointField::FLOAT64) {
    return false;
  }
  const std::uint64_t minimum_row_step =
    static_cast<std::uint64_t>(message.width) * message.point_step;
  const std::uint64_t minimum_data_size =
    static_cast<std::uint64_t>(message.height - 1U) * message.row_step + minimum_row_step;
  if (message.row_step < minimum_row_step || minimum_data_size > message.data.size()) {
    return false;
  }
  std::int64_t minimum = std::numeric_limits<std::int64_t>::max();
  std::int64_t maximum = std::numeric_limits<std::int64_t>::min();
  std::size_t valid_count = 0U;
  for (std::uint32_t row = 0; row < message.height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * message.row_step;
    for (std::uint32_t column = 0; column < message.width; ++column) {
      const std::size_t offset = row_offset + static_cast<std::size_t>(column) * message.point_step;
      if (offset + message.point_step > message.data.size()) {
        return false;
      }
      const std::int64_t stamp_ns = pointStampNs(
        readNumericField(message.data.data() + offset, *field), mode, header_stamp_ns);
      if (stamp_ns < 0) {
        continue;
      }
      minimum = std::min(minimum, stamp_ns);
      maximum = std::max(maximum, stamp_ns);
      ++valid_count;
    }
  }
  if (valid_count == 0U || maximum < minimum) {
    return false;
  }
  const double duration_s = static_cast<double>(maximum - minimum) * 1.0e-9;
  if (!std::isfinite(duration_s) || duration_s < 0.0 || duration_s > max_scan_duration_s) {
    return false;
  }
  const std::int64_t tolerance_ns = static_cast<std::int64_t>(
    std::llround(header_tolerance_s * 1.0e9));
  const std::int64_t max_duration_ns = static_cast<std::int64_t>(
    std::llround(max_scan_duration_s * 1.0e9));
  if (minimum < header_stamp_ns - tolerance_ns ||
    maximum > header_stamp_ns + max_duration_ns + tolerance_ns)
  {
    return false;
  }
  start_stamp_ns = minimum;
  end_stamp_ns = maximum;
  has_point_time = true;
  return true;
}

Eigen::Isometry3d LioOdometry::interpolatePose(
  const std::vector<TimedPose> & trajectory, const std::int64_t stamp_ns)
{
  if (trajectory.empty()) {
    return Eigen::Isometry3d::Identity();
  }
  if (stamp_ns <= trajectory.front().stamp_ns) {
    return trajectory.front().odom_T_base;
  }
  if (stamp_ns >= trajectory.back().stamp_ns) {
    return trajectory.back().odom_T_base;
  }
  const auto upper = std::upper_bound(
    trajectory.begin(), trajectory.end(), stamp_ns,
    [](const std::int64_t value, const TimedPose & pose) {return value < pose.stamp_ns;});
  const auto lower = std::prev(upper);
  const double alpha = static_cast<double>(stamp_ns - lower->stamp_ns) /
    static_cast<double>(upper->stamp_ns - lower->stamp_ns);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = (1.0 - alpha) * lower->odom_T_base.translation() +
    alpha * upper->odom_T_base.translation();
  const Eigen::Quaterniond lower_quaternion(lower->odom_T_base.linear());
  const Eigen::Quaterniond upper_quaternion(upper->odom_T_base.linear());
  result.linear() = lower_quaternion.slerp(alpha, upper_quaternion).normalized().toRotationMatrix();
  return result;
}

double LioOdometry::rotationAngle(const Eigen::Matrix3d & rotation)
{
  return std::acos(std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0));
}

}  // namespace gole::slam_3d_localization
