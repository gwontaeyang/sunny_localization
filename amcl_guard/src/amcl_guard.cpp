#include "amcl_guard/amcl_guard.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <tf2/exceptions.h>

namespace amcl_guard
{

AmclGuard::AmclGuard(const rclcpp::NodeOptions & options)
: Node("amcl_guard", options),
  enabled_(true),
  odom_frame_id_("odom"),
  base_frame_id_("base_link"),
  require_good_pose_(true),
  max_pose_age_(3.0),
  allow_stale_pose_when_stationary_(true),
  stale_pose_max_age_(30.0),
  stale_pose_max_odom_translation_(0.05),
  stale_pose_max_odom_rotation_(0.05),
  tf_lookup_timeout_(0.05),
  max_xy_covariance_(0.5),
  max_yaw_covariance_(0.4),
  min_valid_beam_ratio_(0.2),
  max_near_beam_ratio_(0.8),
  near_range_(0.35),
  valid_min_range_(-1.0),
  valid_max_range_(-1.0),
  max_hold_duration_(3.0),
  drop_scans_after_timeout_(true),
  log_throttle_sec_(1.0),
  publish_status_topics_(false),
  log_status_changes_(true),
  log_blocked_scan_warnings_(true),
  log_level_("info"),
  enable_imu_roughness_gate_(false),
  tilt_threshold_deg_(2.0),
  gyro_xy_threshold_rad_s_(0.05),
  accel_norm_deviation_threshold_m_s2_(0.35),
  tilt_rate_threshold_deg_s_(0.0),
  accel_norm_rate_threshold_m_s3_(0.0),
  max_scan_imu_dt_s_(0.05),
  imu_roughness_hold_duration_s_(0.35),
  imu_roughness_hard_hold_duration_s_(0.0),
  roughness_score_gate_(1.0),
  roughness_score_hard_gate_(2.0),
  roughness_gate_mode_("drop"),
  roughness_throttle_min_publish_period_s_(0.5),
  roughness_throttle_drop_hard_(true),
  roughness_max_continuous_drop_duration_s_(0.0),
  roughness_adaptive_require_return_revisit_(true),
  roughness_adaptive_revisit_radius_m_(0.8),
  roughness_adaptive_min_separation_s_(20.0),
  roughness_adaptive_min_heading_delta_rad_(2.2),
  roughness_adaptive_history_duration_s_(900.0),
  roughness_adaptive_use_travel_heading_(false),
  roughness_adaptive_min_travel_distance_m_(0.25),
  roughness_adaptive_travel_heading_window_s_(4.0),
  roughness_stabilize_max_source_age_s_(0.35),
  roughness_stabilize_max_event_duration_s_(0.5),
  roughness_stabilize_min_publish_period_s_(0.1),
  roughness_stabilize_max_translation_m_(0.2),
  roughness_stabilize_max_yaw_rad_(0.15),
  roughness_stabilize_max_roll_pitch_rad_(0.05),
  roughness_stabilize_min_cached_valid_ratio_(0.2),
  roughness_stabilize_min_output_valid_ratio_(0.15),
  roughness_stabilize_min_output_valid_beams_(20),
  roughness_stabilize_fallback_mode_("drop"),
  roughness_stabilize_require_return_revisit_(false),
  use_nearest_imu_sample_(false),
  imu_buffer_duration_s_(1.0),
  last_imu_tilt_deg_(std::numeric_limits<double>::quiet_NaN()),
  last_imu_accel_norm_deviation_m_s2_(std::numeric_limits<double>::quiet_NaN()),
  last_imu_tilt_rate_deg_s_(std::numeric_limits<double>::quiet_NaN()),
  last_imu_accel_norm_rate_m_s3_(std::numeric_limits<double>::quiet_NaN()),
  roughness_throttle_event_active_(false),
  last_rough_hold_duration_s_(0.0),
  last_rough_hold_hard_(false),
  hold_allowed_(false)
{
  log_level_ = declare_parameter<std::string>("log_level", "info");
  enabled_ = declare_parameter<bool>("enabled", true);
  input_scan_topic_ = declare_parameter<std::string>("input_scan_topic", "/scan_laserscan");
  output_scan_topic_ = declare_parameter<std::string>("output_scan_topic", "/scan_amcl");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
  amcl_pose_topic_ = declare_parameter<std::string>("amcl_pose_topic", "/amcl_pose");
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  require_good_pose_ = declare_parameter<bool>("require_good_pose", true);
  max_pose_age_ = declare_parameter<double>("max_pose_age", 3.0);
  allow_stale_pose_when_stationary_ = declare_parameter<bool>(
    "allow_stale_pose_when_stationary", true);
  stale_pose_max_age_ = declare_parameter<double>("stale_pose_max_age", 30.0);
  stale_pose_max_odom_translation_ = declare_parameter<double>(
    "stale_pose_max_odom_translation", 0.05);
  stale_pose_max_odom_rotation_ = declare_parameter<double>(
    "stale_pose_max_odom_rotation", 0.05);
  tf_lookup_timeout_ = declare_parameter<double>("tf_lookup_timeout", 0.05);
  max_xy_covariance_ = declare_parameter<double>("max_xy_covariance", 0.5);
  max_yaw_covariance_ = declare_parameter<double>("max_yaw_covariance", 0.4);
  min_valid_beam_ratio_ = declare_parameter<double>("min_valid_beam_ratio", 0.2);
  max_near_beam_ratio_ = declare_parameter<double>("max_near_beam_ratio", 0.8);
  near_range_ = declare_parameter<double>("near_range", 0.35);
  valid_min_range_ = declare_parameter<double>("valid_min_range", -1.0);
  valid_max_range_ = declare_parameter<double>("valid_max_range", -1.0);
  max_hold_duration_ = declare_parameter<double>("max_hold_duration", 3.0);
  drop_scans_after_timeout_ = declare_parameter<bool>("drop_scans_after_timeout", true);
  log_throttle_sec_ = declare_parameter<double>("log_throttle_sec", 1.0);
  publish_status_topics_ = declare_parameter<bool>("publish_status_topics", false);
  log_status_changes_ = declare_parameter<bool>("log_status_changes", true);
  log_blocked_scan_warnings_ = declare_parameter<bool>("log_blocked_scan_warnings", true);
  enable_imu_roughness_gate_ = declare_parameter<bool>("enable_imu_roughness_gate", false);
  tilt_threshold_deg_ = declare_parameter<double>("tilt_threshold_deg", 2.0);
  gyro_xy_threshold_rad_s_ = declare_parameter<double>("gyro_xy_threshold_rad_s", 0.05);
  accel_norm_deviation_threshold_m_s2_ = declare_parameter<double>(
    "accel_norm_deviation_threshold_m_s2", 0.35);
  tilt_rate_threshold_deg_s_ = declare_parameter<double>("tilt_rate_threshold_deg_s", 0.0);
  accel_norm_rate_threshold_m_s3_ = declare_parameter<double>(
    "accel_norm_rate_threshold_m_s3", 0.0);
  max_scan_imu_dt_s_ = declare_parameter<double>("max_scan_imu_dt_s", 0.05);
  imu_roughness_hold_duration_s_ = declare_parameter<double>(
    "imu_roughness_hold_duration_s", 0.35);
  imu_roughness_hard_hold_duration_s_ = declare_parameter<double>(
    "imu_roughness_hard_hold_duration_s", 0.0);
  roughness_score_gate_ = declare_parameter<double>("roughness_score_gate", 1.0);
  roughness_score_hard_gate_ = declare_parameter<double>("roughness_score_hard_gate", 2.0);
  roughness_gate_mode_ = declare_parameter<std::string>("roughness_gate_mode", "drop");
  roughness_throttle_min_publish_period_s_ = declare_parameter<double>(
    "roughness_throttle_min_publish_period_s", 0.5);
  roughness_throttle_drop_hard_ = declare_parameter<bool>(
    "roughness_throttle_drop_hard", true);
  roughness_max_continuous_drop_duration_s_ = declare_parameter<double>(
    "roughness_max_continuous_drop_duration_s", 0.0);
  roughness_adaptive_require_return_revisit_ = declare_parameter<bool>(
    "roughness_adaptive_require_return_revisit", true);
  roughness_adaptive_revisit_radius_m_ = declare_parameter<double>(
    "roughness_adaptive_revisit_radius_m", 0.8);
  roughness_adaptive_min_separation_s_ = declare_parameter<double>(
    "roughness_adaptive_min_separation_s", 20.0);
  roughness_adaptive_min_heading_delta_rad_ = declare_parameter<double>(
    "roughness_adaptive_min_heading_delta_rad", 2.2);
  roughness_adaptive_history_duration_s_ = declare_parameter<double>(
    "roughness_adaptive_history_duration_s", 900.0);
  roughness_adaptive_use_travel_heading_ = declare_parameter<bool>(
    "roughness_adaptive_use_travel_heading", false);
  roughness_adaptive_min_travel_distance_m_ = declare_parameter<double>(
    "roughness_adaptive_min_travel_distance_m", 0.25);
  roughness_adaptive_travel_heading_window_s_ = declare_parameter<double>(
    "roughness_adaptive_travel_heading_window_s", 4.0);
  roughness_stabilize_max_source_age_s_ = declare_parameter<double>(
    "roughness_stabilize_max_source_age_s", 0.35);
  roughness_stabilize_max_event_duration_s_ = declare_parameter<double>(
    "roughness_stabilize_max_event_duration_s", 0.5);
  roughness_stabilize_min_publish_period_s_ = declare_parameter<double>(
    "roughness_stabilize_min_publish_period_s", 0.1);
  roughness_stabilize_max_translation_m_ = declare_parameter<double>(
    "roughness_stabilize_max_translation_m", 0.2);
  roughness_stabilize_max_yaw_rad_ = declare_parameter<double>(
    "roughness_stabilize_max_yaw_rad", 0.15);
  roughness_stabilize_max_roll_pitch_rad_ = declare_parameter<double>(
    "roughness_stabilize_max_roll_pitch_rad", 0.05);
  roughness_stabilize_min_cached_valid_ratio_ = declare_parameter<double>(
    "roughness_stabilize_min_cached_valid_ratio", 0.2);
  roughness_stabilize_min_output_valid_ratio_ = declare_parameter<double>(
    "roughness_stabilize_min_output_valid_ratio", 0.15);
  const int roughness_stabilize_min_output_valid_beams = declare_parameter<int>(
    "roughness_stabilize_min_output_valid_beams", 20);
  roughness_stabilize_min_output_valid_beams_ = static_cast<std::size_t>(
    std::max(roughness_stabilize_min_output_valid_beams, 0));
  roughness_stabilize_fallback_mode_ = declare_parameter<std::string>(
    "roughness_stabilize_fallback_mode", "drop");
  roughness_stabilize_require_return_revisit_ = declare_parameter<bool>(
    "roughness_stabilize_require_return_revisit", false);
  use_nearest_imu_sample_ = declare_parameter<bool>("use_nearest_imu_sample", false);
  imu_buffer_duration_s_ = declare_parameter<double>("imu_buffer_duration_s", 1.0);

  if (tilt_rate_threshold_deg_s_ < 0.0) {
    tilt_rate_threshold_deg_s_ = 0.0;
  }
  if (accel_norm_rate_threshold_m_s3_ < 0.0) {
    accel_norm_rate_threshold_m_s3_ = 0.0;
  }
  if (imu_roughness_hard_hold_duration_s_ < 0.0) {
    imu_roughness_hard_hold_duration_s_ = 0.0;
  }
  if (
    roughness_gate_mode_ != "drop" &&
    roughness_gate_mode_ != "throttle" &&
    roughness_gate_mode_ != "observe" &&
    roughness_gate_mode_ != "adaptive" &&
    roughness_gate_mode_ != "map_protect" &&
    roughness_gate_mode_ != "stabilize")
  {
    roughness_gate_mode_ = "drop";
  }
  if (roughness_throttle_min_publish_period_s_ < 0.0) {
    roughness_throttle_min_publish_period_s_ = 0.0;
  }
  if (roughness_max_continuous_drop_duration_s_ < 0.0) {
    roughness_max_continuous_drop_duration_s_ = 0.0;
  }
  if (roughness_adaptive_revisit_radius_m_ < 0.0) {
    roughness_adaptive_revisit_radius_m_ = 0.0;
  }
  if (roughness_adaptive_min_separation_s_ < 0.0) {
    roughness_adaptive_min_separation_s_ = 0.0;
  }
  if (roughness_adaptive_min_heading_delta_rad_ < 0.0) {
    roughness_adaptive_min_heading_delta_rad_ = 0.0;
  }
  constexpr double pi = 3.14159265358979323846;
  if (roughness_adaptive_min_heading_delta_rad_ > pi) {
    roughness_adaptive_min_heading_delta_rad_ = pi;
  }
  if (roughness_adaptive_history_duration_s_ < 0.0) {
    roughness_adaptive_history_duration_s_ = 0.0;
  }
  roughness_adaptive_min_travel_distance_m_ =
    std::max(roughness_adaptive_min_travel_distance_m_, 0.0);
  roughness_adaptive_travel_heading_window_s_ =
    std::max(roughness_adaptive_travel_heading_window_s_, 0.0);
  roughness_stabilize_max_source_age_s_ =
    std::max(roughness_stabilize_max_source_age_s_, 0.0);
  roughness_stabilize_max_event_duration_s_ =
    std::max(roughness_stabilize_max_event_duration_s_, 0.0);
  roughness_stabilize_min_publish_period_s_ =
    std::max(roughness_stabilize_min_publish_period_s_, 0.0);
  roughness_stabilize_max_translation_m_ =
    std::max(roughness_stabilize_max_translation_m_, 0.0);
  roughness_stabilize_max_yaw_rad_ =
    std::max(roughness_stabilize_max_yaw_rad_, 0.0);
  roughness_stabilize_max_roll_pitch_rad_ =
    std::max(roughness_stabilize_max_roll_pitch_rad_, 0.0);
  roughness_stabilize_min_cached_valid_ratio_ =
    std::clamp(roughness_stabilize_min_cached_valid_ratio_, 0.0, 1.0);
  roughness_stabilize_min_output_valid_ratio_ =
    std::clamp(roughness_stabilize_min_output_valid_ratio_, 0.0, 1.0);
  if (
    roughness_stabilize_fallback_mode_ != "drop" &&
    roughness_stabilize_fallback_mode_ != "observe")
  {
    roughness_stabilize_fallback_mode_ = "drop";
  }
  if (imu_buffer_duration_s_ < max_scan_imu_dt_s_) {
    imu_buffer_duration_s_ = max_scan_imu_dt_s_;
  }

  logger_ = gole::logger::Logger::createPackageLogger(
    "amcl_guard",
    gole::logger::Logger::stringToLevel(log_level_));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
    output_scan_topic_, rclcpp::SensorDataQoS());
  if (publish_status_topics_) {
    active_pub_ = create_publisher<std_msgs::msg::Bool>("~/active", rclcpp::QoS(10));
    status_pub_ = create_publisher<std_msgs::msg::String>("~/status", rclcpp::QoS(10));
  }

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    input_scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AmclGuard::scanCallback, this, std::placeholders::_1));
  if (enable_imu_roughness_gate_) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AmclGuard::imuCallback, this, std::placeholders::_1));
  }
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    amcl_pose_topic_, rclcpp::QoS(10),
    std::bind(&AmclGuard::poseCallback, this, std::placeholders::_1));

  logger_->info(
    "AMCL guard forwarding {} -> {}, imu_roughness_gate={}, roughness_gate_mode={}, "
    "nearest_imu={}, max_continuous_drop={:.2f}s, adaptive_return_revisit={}, "
    "adaptive_radius={:.2f}m, "
    "adaptive_min_separation={:.2f}s, adaptive_min_heading_delta={:.2f}rad, "
    "adaptive_heading={}, adaptive_travel={:.2f}m/{:.2f}s, "
    "stabilize_age={:.2f}s, stabilize_motion={:.2f}m/{:.2f}rad, "
    "stabilize_return_revisit={}",
    input_scan_topic_, output_scan_topic_, enable_imu_roughness_gate_ ? "true" : "false",
    roughness_gate_mode_, use_nearest_imu_sample_ ? "true" : "false",
    roughness_max_continuous_drop_duration_s_,
    roughness_adaptive_require_return_revisit_ ? "true" : "false",
    roughness_adaptive_revisit_radius_m_,
    roughness_adaptive_min_separation_s_,
    roughness_adaptive_min_heading_delta_rad_,
    roughness_adaptive_use_travel_heading_ ? "travel" : "pose_yaw",
    roughness_adaptive_min_travel_distance_m_,
    roughness_adaptive_travel_heading_window_s_,
    roughness_stabilize_max_source_age_s_,
    roughness_stabilize_max_translation_m_,
    roughness_stabilize_max_yaw_rad_,
    roughness_stabilize_require_return_revisit_ ? "true" : "false");
}

void AmclGuard::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  const rclcpp::Time stamp = messageTime(msg->header.stamp);
  const double tilt_deg = imuTiltDeg(*msg);
  const auto & acceleration = msg->linear_acceleration;
  const double accel_norm = std::sqrt(
    acceleration.x * acceleration.x +
    acceleration.y * acceleration.y +
    acceleration.z * acceleration.z);
  constexpr double gravity = 9.80665;
  const double accel_dev = std::fabs(accel_norm - gravity);
  const auto & angular_velocity = msg->angular_velocity;
  const double gyro_xy = std::hypot(angular_velocity.x, angular_velocity.y);

  double tilt_rate_deg_s = std::numeric_limits<double>::quiet_NaN();
  double accel_norm_rate_m_s3 = std::numeric_limits<double>::quiet_NaN();
  if (last_imu_time_) {
    const double dt_s = (stamp - *last_imu_time_).seconds();
    if (dt_s > 1.0e-6) {
      if (std::isfinite(tilt_deg) && std::isfinite(last_imu_tilt_deg_)) {
        tilt_rate_deg_s = std::fabs(tilt_deg - last_imu_tilt_deg_) / dt_s;
      }
      if (std::isfinite(accel_dev) && std::isfinite(last_imu_accel_norm_deviation_m_s2_)) {
        accel_norm_rate_m_s3 =
          std::fabs(accel_dev - last_imu_accel_norm_deviation_m_s2_) / dt_s;
      }
    }
  }

  ImuSample sample;
  sample.msg = *msg;
  sample.stamp = stamp;
  sample.tilt_deg = tilt_deg;
  sample.gyro_xy_rad_s = gyro_xy;
  sample.accel_norm_deviation_m_s2 = accel_dev;
  sample.tilt_rate_deg_s = tilt_rate_deg_s;
  sample.accel_norm_rate_m_s3 = accel_norm_rate_m_s3;
  imu_samples_.push_back(sample);
  while (!imu_samples_.empty()) {
    const double age_s = (stamp - imu_samples_.front().stamp).seconds();
    if (age_s <= imu_buffer_duration_s_) {
      break;
    }
    imu_samples_.pop_front();
  }

  last_imu_ = *msg;
  last_imu_time_ = stamp;
  last_imu_tilt_deg_ = tilt_deg;
  last_imu_accel_norm_deviation_m_s2_ = accel_dev;
  last_imu_tilt_rate_deg_s_ = tilt_rate_deg_s;
  last_imu_accel_norm_rate_m_s3_ = accel_norm_rate_m_s3;
}

void AmclGuard::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  last_pose_ = *msg;
  last_pose_odom_pose_ = lookupOdomPose(rclcpp::Time(msg->header.stamp));
  if (!last_pose_odom_pose_) {
    last_pose_odom_pose_ = lookupLatestOdomPose();
  }
}

void AmclGuard::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!enabled_) {
    endHold("disabled");
    resetRoughnessThrottleState();
    resetStabilizeEvent();
    scan_pub_->publish(*msg);
    return;
  }

  const ScanStats stats = scanStats(*msg);
  const bool occluded =
    stats.valid_ratio<min_valid_beam_ratio_ || stats.near_ratio> max_near_beam_ratio_;
  const auto roughness = roughnessStats(*msg);
  const rclcpp::Time scan_time = messageTime(msg->header.stamp);
  std::optional<OdomPose> scan_odom_pose;
  std::optional<bool> adaptive_return_revisit_risk;
  const bool needs_return_revisit_state =
    roughness_gate_mode_ == "adaptive" ||
    roughness_gate_mode_ == "map_protect" ||
    (roughness_gate_mode_ == "stabilize" &&
    roughness_stabilize_require_return_revisit_);
  if (needs_return_revisit_state) {
    scan_odom_pose = lookupOdomPose(scan_time);
    if (!scan_odom_pose) {
      scan_odom_pose = lookupLatestOdomPose();
    }
    if (scan_odom_pose) {
      adaptive_return_revisit_risk =
        adaptiveReturnRevisitRisk(scan_time, *scan_odom_pose);
      rememberAdaptiveOdomPose(scan_time, *scan_odom_pose);
    } else {
      adaptive_return_revisit_risk = false;
    }
  }
  const bool imu_rough =
    roughness && roughness->active;
  const bool guarded = occluded || imu_rough;
  const std::string guard_reason = occluded ? "occluded" : "imu_roughness";

  if (!guarded) {
    endHold("clear");
    rememberStableScan(*msg, scan_time, stats, roughness);
    resetStabilizeEvent();
    if (roughness_throttle_event_active_) {
      publishStatus(false, "imu_roughness_recovery_allowed", stats, roughness, 0.0);
    }
    resetRoughnessThrottleState();
    scan_pub_->publish(*msg);
    return;
  }

  if (!occluded && imu_rough && roughness_gate_mode_ == "stabilize") {
    endHold("roughness_stabilize");
    resetRoughnessThrottleState();
    if (
      roughness_stabilize_require_return_revisit_ &&
      !adaptive_return_revisit_risk.value_or(false))
    {
      resetStabilizeEvent();
      const std::string status = scan_odom_pose ?
        "imu_roughness_stabilize_observed_no_return_revisit" :
        "imu_roughness_stabilize_observed_no_odom";
      publishStatus(false, status, stats, roughness, 0.0);
      scan_pub_->publish(*msg);
      return;
    }

    std::string failure_reason;
    auto stabilized = makeStabilizedScan(*msg, scan_time, failure_reason);
    if (stabilized) {
      publishStatus(
        false, "imu_roughness_stabilized_scan_allowed", stats, roughness, 0.0);
      scan_pub_->publish(stabilized->scan);
      return;
    }

    const std::string status =
      "imu_roughness_stabilize_" + failure_reason +
      (roughness_stabilize_fallback_mode_ == "observe" ? "_observed" : "_dropped");
    const bool should_drop = roughness_stabilize_fallback_mode_ != "observe";
    publishStatus(should_drop, status, stats, roughness, 0.0);
    if (!should_drop) {
      scan_pub_->publish(*msg);
      return;
    }
    throttledWarn(
      get_clock()->now(),
      "AMCL scan blocked: status=" + status);
    return;
  }

  if (!occluded && imu_rough && roughness_gate_mode_ == "adaptive") {
    const bool revisit_risk = adaptive_return_revisit_risk.value_or(
      !roughness_adaptive_require_return_revisit_);
    if (!revisit_risk) {
      endHold("roughness_adaptive_observe");
      resetRoughnessThrottleState();
      const std::string status = scan_odom_pose ?
        "imu_roughness_adaptive_observed_no_return_revisit" :
        "imu_roughness_adaptive_observed_no_odom";
      publishStatus(false, status, stats, roughness, 0.0);
      scan_pub_->publish(*msg);
      return;
    }

    endHold("roughness_adaptive_return_revisit");
    const auto decision = roughnessThrottleDecision(scan_time, roughness);
    const bool should_publish = decision != RoughnessThrottleDecision::kDropped;
    const std::string status =
      decision == RoughnessThrottleDecision::kBridgeAllowed ?
      "imu_roughness_adaptive_throttle_bridge_allowed" :
      (should_publish ?
      "imu_roughness_adaptive_throttle_allowed" :
      "imu_roughness_adaptive_throttle_dropped");
    publishStatus(!should_publish, status, stats, roughness, 0.0);
    if (should_publish) {
      scan_pub_->publish(*msg);
      return;
    }

    std::ostringstream warn_text;
    warn_text << std::fixed << std::setprecision(2)
              << "AMCL scan blocked: status=" << status
              << ", adaptive_return_revisit=true"
              << ", valid_ratio=" << stats.valid_ratio
              << ", near_ratio=" << stats.near_ratio;
    if (roughness) {
      warn_text << ", rough_score=" << std::setprecision(2) << roughness->score
                << ", hard=" << (roughness->hard ? "true" : "false")
                << ", hold=" << (roughness->hold_active ? "true" : "false")
                << ", imu_age=" << roughness->age_s << "s";
    }
    throttledWarn(get_clock()->now(), warn_text.str());
    return;
  }

  if (!occluded && imu_rough && roughness_gate_mode_ == "map_protect") {
    const bool revisit_risk = adaptive_return_revisit_risk.value_or(false);
    if (revisit_risk) {
      endHold("roughness_map_protect_return_revisit");
      resetRoughnessThrottleState();
      publishStatus(
        false, "imu_roughness_map_protect_return_revisit_allowed", stats, roughness, 0.0);
      scan_pub_->publish(*msg);
      return;
    }

    endHold("roughness_map_protect_first_pass");
    const auto decision = roughnessThrottleDecision(scan_time, roughness);
    const bool should_publish = decision != RoughnessThrottleDecision::kDropped;
    const std::string status =
      decision == RoughnessThrottleDecision::kBridgeAllowed ?
      "imu_roughness_map_protect_throttle_bridge_allowed" :
      (should_publish ?
      "imu_roughness_map_protect_throttle_allowed" :
      "imu_roughness_map_protect_throttle_dropped");
    publishStatus(!should_publish, status, stats, roughness, 0.0);
    if (should_publish) {
      scan_pub_->publish(*msg);
      return;
    }

    std::ostringstream warn_text;
    warn_text << std::fixed << std::setprecision(2)
              << "AMCL scan blocked: status=" << status
              << ", map_protect_return_revisit=false"
              << ", valid_ratio=" << stats.valid_ratio
              << ", near_ratio=" << stats.near_ratio;
    if (roughness) {
      warn_text << ", rough_score=" << std::setprecision(2) << roughness->score
                << ", hard=" << (roughness->hard ? "true" : "false")
                << ", hold=" << (roughness->hold_active ? "true" : "false")
                << ", imu_age=" << roughness->age_s << "s";
    }
    throttledWarn(get_clock()->now(), warn_text.str());
    return;
  }

  if (!occluded && imu_rough && roughness_gate_mode_ == "observe") {
    endHold("roughness_observe");
    resetRoughnessThrottleState();
    publishStatus(false, "imu_roughness_observed", stats, roughness, 0.0);
    scan_pub_->publish(*msg);
    return;
  }

  if (!occluded && imu_rough && roughness_gate_mode_ == "throttle") {
    endHold("roughness_throttle");
    const auto decision = roughnessThrottleDecision(scan_time, roughness);
    const bool should_publish = decision != RoughnessThrottleDecision::kDropped;
    const std::string status =
      decision == RoughnessThrottleDecision::kBridgeAllowed ?
      "imu_roughness_throttle_bridge_allowed" :
      (should_publish ?
      "imu_roughness_throttle_allowed" :
      "imu_roughness_throttle_dropped");
    publishStatus(!should_publish, status, stats, roughness, 0.0);
    if (should_publish) {
      scan_pub_->publish(*msg);
      return;
    }

    std::ostringstream warn_text;
    warn_text << std::fixed << std::setprecision(2)
              << "AMCL scan blocked: status=" << status
              << ", valid_ratio=" << stats.valid_ratio
              << ", near_ratio=" << stats.near_ratio;
    if (roughness) {
      warn_text << ", rough_score=" << std::setprecision(2) << roughness->score
                << ", hard=" << (roughness->hard ? "true" : "false")
                << ", hold=" << (roughness->hold_active ? "true" : "false")
                << ", imu_age=" << roughness->age_s << "s";
    }
    throttledWarn(get_clock()->now(), warn_text.str());
    return;
  }

  const rclcpp::Time now = get_clock()->now();
  if (!hold_start_time_) {
    hold_start_time_ = now;
    hold_allowed_ = hasGoodPose(now);
  }

  if (!hold_allowed_) {
    publishStatus(false, guard_reason + "_without_good_pose", stats, roughness, 0.0);
    scan_pub_->publish(*msg);
    return;
  }

  const double elapsed = (now - *hold_start_time_).seconds();
  const bool timed_out = elapsed > max_hold_duration_;
  const bool should_drop = !timed_out || drop_scans_after_timeout_;
  const std::string status = guard_reason + (timed_out ? "_timeout" : "_holding");

  publishStatus(true, status, stats, roughness, elapsed);

  std::ostringstream warn_text;
  warn_text << std::fixed << std::setprecision(2)
            << "AMCL scan blocked: status=" << status
            << ", elapsed=" << elapsed << "s"
            << ", valid_ratio=" << stats.valid_ratio
            << ", near_ratio=" << stats.near_ratio;
  if (roughness) {
    warn_text << ", rough_score=" << std::setprecision(2) << roughness->score
              << ", tilt=" << roughness->tilt_deg << "deg"
              << ", gyro_xy=" << roughness->gyro_xy_rad_s << "rad/s"
              << ", accel_dev=" << roughness->accel_norm_deviation_m_s2 << "m/s^2"
              << ", tilt_rate=" << roughness->tilt_rate_deg_s << "deg/s"
              << ", accel_rate=" << roughness->accel_norm_rate_m_s3 << "m/s^3"
              << ", imu_age=" << roughness->age_s << "s";
  }
  throttledWarn(now, warn_text.str());

  if (!should_drop) {
    scan_pub_->publish(*msg);
  }
}

AmclGuard::ScanStats AmclGuard::scanStats(
  const sensor_msgs::msg::LaserScan & msg) const
{
  ScanStats stats;
  stats.total = msg.ranges.size();

  if (stats.total == 0) {
    return stats;
  }

  const double min_range = valid_min_range_ > 0.0 ? valid_min_range_ : msg.range_min;
  const double max_range = valid_max_range_ > 0.0 ? valid_max_range_ : msg.range_max;

  for (const auto scan_range : msg.ranges) {
    if (!std::isfinite(scan_range)) {
      continue;
    }
    if (scan_range < min_range || scan_range > max_range) {
      continue;
    }

    ++stats.valid;
    if (scan_range <= near_range_) {
      ++stats.near;
    }
  }

  stats.valid_ratio = static_cast<double>(stats.valid) / static_cast<double>(stats.total);
  stats.near_ratio = stats.valid > 0 ?
    static_cast<double>(stats.near) / static_cast<double>(stats.valid) : 1.0;

  return stats;
}

std::optional<AmclGuard::RoughnessStats> AmclGuard::roughnessStats(
  const sensor_msgs::msg::LaserScan & msg)
{
  if (!enable_imu_roughness_gate_) {
    return std::nullopt;
  }

  const rclcpp::Time scan_time = messageTime(msg.header.stamp);

  std::optional<ImuSample> sample;
  if (use_nearest_imu_sample_) {
    sample = nearestImuSample(scan_time);
  } else if (last_imu_ && last_imu_time_) {
    ImuSample latest;
    latest.msg = *last_imu_;
    latest.stamp = *last_imu_time_;
    latest.tilt_deg = last_imu_tilt_deg_;
    latest.accel_norm_deviation_m_s2 = last_imu_accel_norm_deviation_m_s2_;
    latest.tilt_rate_deg_s = last_imu_tilt_rate_deg_s_;
    latest.accel_norm_rate_m_s3 = last_imu_accel_norm_rate_m_s3_;
    const auto & angular_velocity = last_imu_->angular_velocity;
    latest.gyro_xy_rad_s = std::hypot(angular_velocity.x, angular_velocity.y);
    sample = latest;
  }
  if (!sample) {
    return std::nullopt;
  }

  const double age_s = std::fabs((scan_time - sample->stamp).seconds());
  if (age_s > max_scan_imu_dt_s_) {
    return std::nullopt;
  }

  const double tilt_deg = sample->tilt_deg;
  const double gyro_xy = sample->gyro_xy_rad_s;
  const double accel_dev = sample->accel_norm_deviation_m_s2;
  const double tilt_rate = sample->tilt_rate_deg_s;
  const double accel_rate = sample->accel_norm_rate_m_s3;
  const double score = roughnessScore(tilt_deg, gyro_xy, accel_dev, tilt_rate, accel_rate);
  const bool score_active =
    std::isfinite(score) &&
    score >= roughness_score_gate_;
  const bool hard_active =
    std::isfinite(score) &&
    score >= roughness_score_hard_gate_;

  if (score_active) {
    last_rough_scan_time_ = scan_time;
    last_rough_hold_duration_s_ =
      hard_active && imu_roughness_hard_hold_duration_s_ > 0.0 ?
      imu_roughness_hard_hold_duration_s_ :
      imu_roughness_hold_duration_s_;
    last_rough_hold_hard_ = hard_active;
  }

  bool hold_active = false;
  if (last_rough_scan_time_) {
    const double hold_age = (scan_time - *last_rough_scan_time_).seconds();
    hold_active =
      hold_age >= 0.0 &&
      last_rough_hold_duration_s_ > 0.0 &&
      hold_age <= last_rough_hold_duration_s_;
  }

  RoughnessStats stats;
  stats.tilt_deg = tilt_deg;
  stats.gyro_xy_rad_s = gyro_xy;
  stats.accel_norm_deviation_m_s2 = accel_dev;
  stats.tilt_rate_deg_s = tilt_rate;
  stats.accel_norm_rate_m_s3 = accel_rate;
  stats.score = score;
  stats.age_s = age_s;
  stats.active = score_active || hold_active;
  stats.hard = hard_active || (hold_active && last_rough_hold_hard_);
  stats.hold_active = hold_active && !score_active;
  return stats;
}

std::optional<AmclGuard::ImuSample> AmclGuard::nearestImuSample(
  const rclcpp::Time & scan_time) const
{
  if (imu_samples_.empty()) {
    return std::nullopt;
  }

  const ImuSample * best = nullptr;
  double best_abs_dt_s = std::numeric_limits<double>::infinity();
  for (const auto & sample : imu_samples_) {
    const double dt_s = (scan_time - sample.stamp).seconds();
    const double abs_dt_s = std::fabs(dt_s);
    if (abs_dt_s < best_abs_dt_s) {
      best_abs_dt_s = abs_dt_s;
      best = &sample;
    }
  }
  if (!best || best_abs_dt_s > max_scan_imu_dt_s_) {
    return std::nullopt;
  }
  return *best;
}

double AmclGuard::roughnessScore(
  const double tilt_deg,
  const double gyro_xy_rad_s,
  const double accel_norm_deviation_m_s2,
  const double tilt_rate_deg_s,
  const double accel_norm_rate_m_s3) const
{
  double score = 0.0;
  bool has_score = false;
  if (tilt_threshold_deg_ > 0.0 && std::isfinite(tilt_deg)) {
    score = std::max(score, tilt_deg / tilt_threshold_deg_);
    has_score = true;
  }
  if (gyro_xy_threshold_rad_s_ > 0.0 && std::isfinite(gyro_xy_rad_s)) {
    score = std::max(score, gyro_xy_rad_s / gyro_xy_threshold_rad_s_);
    has_score = true;
  }
  if (
    accel_norm_deviation_threshold_m_s2_ > 0.0 &&
    std::isfinite(accel_norm_deviation_m_s2))
  {
    score = std::max(
      score,
      accel_norm_deviation_m_s2 / accel_norm_deviation_threshold_m_s2_);
    has_score = true;
  }
  if (tilt_rate_threshold_deg_s_ > 0.0 && std::isfinite(tilt_rate_deg_s)) {
    score = std::max(score, tilt_rate_deg_s / tilt_rate_threshold_deg_s_);
    has_score = true;
  }
  if (accel_norm_rate_threshold_m_s3_ > 0.0 && std::isfinite(accel_norm_rate_m_s3)) {
    score = std::max(score, accel_norm_rate_m_s3 / accel_norm_rate_threshold_m_s3_);
    has_score = true;
  }
  return has_score ? score : std::numeric_limits<double>::quiet_NaN();
}

double AmclGuard::imuTiltDeg(const sensor_msgs::msg::Imu & msg) const
{
  const auto & q = msg.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm <= 1.0e-12) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);
  const double sinp = 2.0 * (w * y - z * x);
  const double pitch = std::fabs(sinp) >= 1.0 ?
    std::copysign(3.14159265358979323846 / 2.0, sinp) :
    std::asin(sinp);
  return std::sqrt(roll * roll + pitch * pitch) * 180.0 / 3.14159265358979323846;
}

rclcpp::Time AmclGuard::messageTime(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec == 0 && stamp.nanosec == 0) {
    return get_clock()->now();
  }
  return rclcpp::Time(stamp, get_clock()->get_clock_type());
}

bool AmclGuard::hasGoodPose(const rclcpp::Time & now)
{
  if (!require_good_pose_) {
    return true;
  }
  if (!last_pose_) {
    return false;
  }

  const rclcpp::Time pose_time(last_pose_->header.stamp);
  const double age = (now - pose_time).seconds();
  if (age < 0.0) {
    return false;
  }

  const auto & cov = last_pose_->pose.covariance;
  const bool covariance_ok =
    cov[0] <= max_xy_covariance_ &&
    cov[7] <= max_xy_covariance_ &&
    cov[35] <= max_yaw_covariance_;
  if (!covariance_ok) {
    return false;
  }

  if (age <= max_pose_age_) {
    return true;
  }

  return canUseStalePose(now, age);
}

bool AmclGuard::canUseStalePose(const rclcpp::Time &, const double pose_age)
{
  if (!allow_stale_pose_when_stationary_) {
    return false;
  }
  if (stale_pose_max_age_ > 0.0 && pose_age > stale_pose_max_age_) {
    return false;
  }
  if (!last_pose_odom_pose_) {
    return false;
  }

  const auto current_odom_pose = lookupLatestOdomPose();
  if (!current_odom_pose) {
    return false;
  }

  const double dx = current_odom_pose->x - last_pose_odom_pose_->x;
  const double dy = current_odom_pose->y - last_pose_odom_pose_->y;
  const double translation = std::hypot(dx, dy);
  const double rotation = std::fabs(
    normalizeAngle(current_odom_pose->yaw - last_pose_odom_pose_->yaw));

  return
    translation <= stale_pose_max_odom_translation_ &&
    rotation <= stale_pose_max_odom_rotation_;
}

std::optional<AmclGuard::OdomPose> AmclGuard::lookupOdomPose(
  const rclcpp::Time & stamp) const
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      odom_frame_id_, base_frame_id_, stamp,
      rclcpp::Duration::from_seconds(tf_lookup_timeout_));
    return transformToOdomPose(transform);
  } catch (const tf2::TransformException & ex) {
    logger_->debug(
      "Failed to lookup {} -> {} at pose stamp: {}",
      odom_frame_id_, base_frame_id_, ex.what());
    return std::nullopt;
  }
}

std::optional<AmclGuard::OdomPose> AmclGuard::lookupLatestOdomPose() const
{
  try {
    const rclcpp::Time latest_time(0, 0, get_clock()->get_clock_type());
    const auto transform = tf_buffer_->lookupTransform(
      odom_frame_id_, base_frame_id_, latest_time,
      rclcpp::Duration::from_seconds(tf_lookup_timeout_));
    return transformToOdomPose(transform);
  } catch (const tf2::TransformException & ex) {
    logger_->debug(
      "Failed to lookup latest {} -> {}: {}",
      odom_frame_id_, base_frame_id_, ex.what());
    return std::nullopt;
  }
}

AmclGuard::OdomPose AmclGuard::transformToOdomPose(
  const geometry_msgs::msg::TransformStamped & transform) const
{
  const auto & translation = transform.transform.translation;
  const auto & q = transform.transform.rotation;

  OdomPose pose;
  pose.x = translation.x;
  pose.y = translation.y;
  pose.yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return pose;
}

double AmclGuard::normalizeAngle(double angle) const
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {
    angle -= 2.0 * pi;
  }
  while (angle < -pi) {
    angle += 2.0 * pi;
  }
  return angle;
}

std::optional<double> AmclGuard::adaptiveTravelHeading(
  const rclcpp::Time & stamp,
  const OdomPose & pose) const
{
  if (
    roughness_adaptive_min_travel_distance_m_ <= 0.0 ||
    roughness_adaptive_travel_heading_window_s_ <= 0.0)
  {
    return std::nullopt;
  }

  for (auto it = adaptive_odom_history_.rbegin(); it != adaptive_odom_history_.rend(); ++it) {
    const double dt_s = (stamp - it->stamp).seconds();
    if (dt_s < 0.0) {
      continue;
    }
    if (dt_s > roughness_adaptive_travel_heading_window_s_) {
      break;
    }

    const double dx = pose.x - it->pose.x;
    const double dy = pose.y - it->pose.y;
    if (std::hypot(dx, dy) >= roughness_adaptive_min_travel_distance_m_) {
      return std::atan2(dy, dx);
    }
  }
  return std::nullopt;
}

bool AmclGuard::adaptiveReturnRevisitRisk(
  const rclcpp::Time & stamp,
  const OdomPose & pose) const
{
  if (!roughness_adaptive_require_return_revisit_) {
    return true;
  }
  if (roughness_adaptive_revisit_radius_m_ <= 0.0 || adaptive_odom_history_.empty()) {
    return false;
  }

  const auto current_travel_heading = roughness_adaptive_use_travel_heading_ ?
    adaptiveTravelHeading(stamp, pose) : std::optional<double>{pose.yaw};
  if (!current_travel_heading) {
    return false;
  }

  const double radius_sq =
    roughness_adaptive_revisit_radius_m_ * roughness_adaptive_revisit_radius_m_;
  for (auto it = adaptive_odom_history_.rbegin(); it != adaptive_odom_history_.rend(); ++it) {
    const double dt_s = (stamp - it->stamp).seconds();
    if (dt_s < 0.0) {
      continue;
    }
    if (
      roughness_adaptive_history_duration_s_ > 0.0 &&
      dt_s > roughness_adaptive_history_duration_s_)
    {
      break;
    }
    if (dt_s < roughness_adaptive_min_separation_s_) {
      continue;
    }

    const double dx = pose.x - it->pose.x;
    const double dy = pose.y - it->pose.y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq > radius_sq) {
      continue;
    }

    const auto previous_heading = roughness_adaptive_use_travel_heading_ ?
      it->travel_heading_rad : std::optional<double>{it->pose.yaw};
    if (!previous_heading) {
      continue;
    }
    const double heading_delta =
      std::fabs(normalizeAngle(*current_travel_heading - *previous_heading));
    if (heading_delta >= roughness_adaptive_min_heading_delta_rad_) {
      return true;
    }
  }
  return false;
}

void AmclGuard::rememberAdaptiveOdomPose(
  const rclcpp::Time & stamp,
  const OdomPose & pose)
{
  if (!adaptive_odom_history_.empty()) {
    const double dt_from_last = (stamp - adaptive_odom_history_.back().stamp).seconds();
    if (dt_from_last < -1.0) {
      adaptive_odom_history_.clear();
    } else if (dt_from_last >= 0.0 && dt_from_last < 0.10) {
      return;
    }
  }

  adaptive_odom_history_.push_back(
    OdomHistorySample{stamp, pose, adaptiveTravelHeading(stamp, pose)});
  while (!adaptive_odom_history_.empty() && roughness_adaptive_history_duration_s_ > 0.0) {
    const double age_s = (stamp - adaptive_odom_history_.front().stamp).seconds();
    if (age_s <= roughness_adaptive_history_duration_s_) {
      break;
    }
    adaptive_odom_history_.pop_front();
  }
}

AmclGuard::RoughnessThrottleDecision AmclGuard::roughnessThrottleDecision(
  const rclcpp::Time & scan_time,
  const std::optional<RoughnessStats> & roughness)
{
  if (!roughness_throttle_event_active_) {
    roughness_throttle_event_active_ = true;
    roughness_drop_start_time_ = scan_time;
  }

  if (roughness_drop_start_time_) {
    const double drop_elapsed_s = (scan_time - *roughness_drop_start_time_).seconds();
    if (drop_elapsed_s < 0.0) {
      last_roughness_publish_time_.reset();
      roughness_drop_start_time_ = scan_time;
    } else if (
      roughness_max_continuous_drop_duration_s_ > 0.0 &&
      drop_elapsed_s >= roughness_max_continuous_drop_duration_s_)
    {
      last_roughness_publish_time_ = scan_time;
      roughness_drop_start_time_ = scan_time;
      return RoughnessThrottleDecision::kBridgeAllowed;
    }
  }

  if (
    roughness &&
    roughness->hard &&
    roughness_throttle_drop_hard_)
  {
    return RoughnessThrottleDecision::kDropped;
  }

  if (!last_roughness_publish_time_) {
    last_roughness_publish_time_ = scan_time;
    roughness_drop_start_time_ = scan_time;
    return RoughnessThrottleDecision::kAllowed;
  }

  const double elapsed_s = (scan_time - *last_roughness_publish_time_).seconds();
  if (elapsed_s < 0.0 || elapsed_s >= roughness_throttle_min_publish_period_s_) {
    last_roughness_publish_time_ = scan_time;
    roughness_drop_start_time_ = scan_time;
    return RoughnessThrottleDecision::kAllowed;
  }

  return RoughnessThrottleDecision::kDropped;
}

void AmclGuard::rememberStableScan(
  const sensor_msgs::msg::LaserScan & scan,
  const rclcpp::Time & scan_time,
  const ScanStats & stats,
  const std::optional<RoughnessStats> & roughness)
{
  if (
    roughness_gate_mode_ != "stabilize" ||
    !roughness ||
    stats.valid_ratio < roughness_stabilize_min_cached_valid_ratio_)
  {
    return;
  }

  if (last_stable_scan_time_ && scan_time < *last_stable_scan_time_) {
    last_stable_scan_.reset();
    last_stable_scan_time_.reset();
    resetStabilizeEvent();
  }

  last_stable_scan_ = scan;
  last_stable_scan_time_ = scan_time;
}

std::optional<StabilizedScan> AmclGuard::makeStabilizedScan(
  const sensor_msgs::msg::LaserScan & current_scan,
  const rclcpp::Time & scan_time,
  std::string & failure_reason)
{
  if (!last_stable_scan_ || !last_stable_scan_time_) {
    failure_reason = "no_stable_scan";
    return std::nullopt;
  }

  const double source_age_s = (scan_time - *last_stable_scan_time_).seconds();
  if (source_age_s < 0.0) {
    last_stable_scan_.reset();
    last_stable_scan_time_.reset();
    resetStabilizeEvent();
    failure_reason = "clock_reset";
    return std::nullopt;
  }
  if (
    roughness_stabilize_max_source_age_s_ > 0.0 &&
    source_age_s > roughness_stabilize_max_source_age_s_)
  {
    failure_reason = "source_too_old";
    return std::nullopt;
  }

  if (!stabilize_event_start_time_) {
    stabilize_event_start_time_ = scan_time;
  }
  const double event_duration_s = (scan_time - *stabilize_event_start_time_).seconds();
  if (event_duration_s < 0.0) {
    resetStabilizeEvent();
    stabilize_event_start_time_ = scan_time;
    failure_reason = "clock_reset";
    return std::nullopt;
  }
  if (
    roughness_stabilize_max_event_duration_s_ > 0.0 &&
    event_duration_s > roughness_stabilize_max_event_duration_s_)
  {
    failure_reason = "event_too_long";
    return std::nullopt;
  }

  if (last_stabilized_publish_time_) {
    const double publish_age_s =
      (scan_time - *last_stabilized_publish_time_).seconds();
    if (
      publish_age_s >= 0.0 &&
      publish_age_s < roughness_stabilize_min_publish_period_s_)
    {
      failure_reason = "rate_limited";
      return std::nullopt;
    }
  }

  if (
    current_scan.header.frame_id.empty() ||
    last_stable_scan_->header.frame_id.empty())
  {
    failure_reason = "missing_frame";
    return std::nullopt;
  }

  geometry_msgs::msg::TransformStamped current_from_source;
  try {
    current_from_source = tf_buffer_->lookupTransform(
      current_scan.header.frame_id,
      scan_time,
      last_stable_scan_->header.frame_id,
      *last_stable_scan_time_,
      odom_frame_id_,
      rclcpp::Duration::from_seconds(tf_lookup_timeout_));
  } catch (const tf2::TransformException & ex) {
    logger_->debug("Failed exact-time stable scan transform: {}", ex.what());
    failure_reason = "tf_failed";
    return std::nullopt;
  }

  const RelativeMotion motion = relativeMotion(current_from_source);
  if (!motion.finite) {
    failure_reason = "invalid_transform";
    return std::nullopt;
  }
  if (motion.translation_m > roughness_stabilize_max_translation_m_) {
    failure_reason = "translation_limit";
    return std::nullopt;
  }
  if (std::fabs(motion.yaw_rad) > roughness_stabilize_max_yaw_rad_) {
    failure_reason = "yaw_limit";
    return std::nullopt;
  }
  if (motion.roll_pitch_rad > roughness_stabilize_max_roll_pitch_rad_) {
    failure_reason = "roll_pitch_limit";
    return std::nullopt;
  }

  const std::size_t ratio_minimum = static_cast<std::size_t>(std::ceil(
      roughness_stabilize_min_output_valid_ratio_ *
      static_cast<double>(current_scan.ranges.size())));
  const std::size_t min_valid_beams =
    std::max(roughness_stabilize_min_output_valid_beams_, ratio_minimum);
  auto stabilized = warpLaserScan(
    *last_stable_scan_,
    current_scan,
    current_from_source,
    min_valid_beams);
  if (!stabilized) {
    failure_reason = "sparse_output";
    return std::nullopt;
  }

  last_stabilized_publish_time_ = scan_time;
  logger_->debug(
    "Published stabilized scan: source_age={:.3f}s event_age={:.3f}s "
    "motion={:.3f}m/{:.3f}rad roll_pitch={:.3f}rad valid_beams={}",
    source_age_s,
    event_duration_s,
    motion.translation_m,
    motion.yaw_rad,
    motion.roll_pitch_rad,
    stabilized->valid_beam_count);
  return stabilized;
}

void AmclGuard::resetStabilizeEvent()
{
  stabilize_event_start_time_.reset();
  last_stabilized_publish_time_.reset();
}

void AmclGuard::resetRoughnessThrottleState()
{
  last_roughness_publish_time_.reset();
  roughness_drop_start_time_.reset();
  roughness_throttle_event_active_ = false;
}

void AmclGuard::endHold(const std::string & reason)
{
  if (hold_start_time_) {
    publishStatus(false, reason, std::nullopt, std::nullopt, 0.0);
  }
  hold_start_time_.reset();
  hold_allowed_ = false;
}

void AmclGuard::publishStatus(
  const bool active,
  const std::string & reason,
  const std::optional<ScanStats> & stats,
  const std::optional<RoughnessStats> & roughness,
  const double elapsed)
{
  std_msgs::msg::Bool active_msg;
  active_msg.data = active;

  std_msgs::msg::String status_msg;
  if (!stats) {
    status_msg.data = reason;
  } else {
    std::ostringstream text;
    text << reason
         << ": elapsed=" << std::fixed << std::setprecision(2) << elapsed << "s"
         << " valid=" << stats->valid << "/" << stats->total
         << " valid_ratio=" << std::setprecision(3) << stats->valid_ratio
         << " near_ratio=" << std::setprecision(3) << stats->near_ratio;
    if (roughness) {
      text << " rough_score=" << std::setprecision(3) << roughness->score
           << " tilt_deg=" << std::setprecision(2) << roughness->tilt_deg
           << " gyro_xy=" << std::setprecision(3) << roughness->gyro_xy_rad_s
           << " accel_dev=" << std::setprecision(3)
           << roughness->accel_norm_deviation_m_s2
           << " tilt_rate_deg_s=" << std::setprecision(3)
           << roughness->tilt_rate_deg_s
           << " accel_rate_m_s3=" << std::setprecision(3)
           << roughness->accel_norm_rate_m_s3
           << " imu_age=" << std::setprecision(3) << roughness->age_s;
    }
    status_msg.data = text.str();
  }

  if (publish_status_topics_ && active_pub_ && status_pub_) {
    active_pub_->publish(active_msg);
    status_pub_->publish(status_msg);
  }
  logStatusChange(active, reason, status_msg.data);
}

void AmclGuard::logStatusChange(
  const bool active,
  const std::string & reason,
  const std::string & status_text)
{
  if (!log_status_changes_) {
    return;
  }

  if (
    last_logged_status_active_ &&
    *last_logged_status_active_ == active &&
    last_logged_status_reason_ == reason)
  {
    return;
  }

  last_logged_status_active_ = active;
  last_logged_status_reason_ = reason;

  if (active) {
    logger_->warn("AMCL guard active: {}", status_text);
  } else {
    logger_->info("AMCL guard inactive: {}", status_text);
  }
}

void AmclGuard::throttledWarn(const rclcpp::Time & now, const std::string & text)
{
  if (!log_blocked_scan_warnings_) {
    return;
  }

  if (last_log_time_) {
    const double elapsed = (now - *last_log_time_).seconds();
    if (elapsed < log_throttle_sec_) {
      return;
    }
  }

  last_log_time_ = now;
  logger_->warn("{}", text);
}

}  // namespace amcl_guard
