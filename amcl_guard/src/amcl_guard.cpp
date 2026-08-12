#include "amcl_guard/amcl_guard.hpp"

#include <cmath>
#include <functional>
#include <iomanip>
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
  hold_allowed_(false)
{
  log_level_ = declare_parameter<std::string>("log_level", "info");
  enabled_ = declare_parameter<bool>("enabled", true);
  input_scan_topic_ = declare_parameter<std::string>("input_scan_topic", "/scan_laserscan");
  output_scan_topic_ = declare_parameter<std::string>("output_scan_topic", "/scan_amcl");
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
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    amcl_pose_topic_, rclcpp::QoS(10),
    std::bind(&AmclGuard::poseCallback, this, std::placeholders::_1));

  logger_->info(
    "AMCL guard forwarding {} -> {}",
    input_scan_topic_, output_scan_topic_);
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
    scan_pub_->publish(*msg);
    return;
  }

  const ScanStats stats = scanStats(*msg);
  const bool occluded =
    (stats.valid_ratio < min_valid_beam_ratio_) ||
    (stats.near_ratio > max_near_beam_ratio_);

  if (!occluded) {
    endHold("clear");
    scan_pub_->publish(*msg);
    return;
  }

  const rclcpp::Time now = get_clock()->now();
  if (!hold_start_time_) {
    hold_start_time_ = now;
    hold_allowed_ = hasGoodPose(now);
  }

  if (!hold_allowed_) {
    publishStatus(false, "occluded_without_good_pose", stats, 0.0);
    scan_pub_->publish(*msg);
    return;
  }

  const double elapsed = (now - *hold_start_time_).seconds();
  const bool timed_out = elapsed > max_hold_duration_;
  const bool should_drop = !timed_out || drop_scans_after_timeout_;
  const std::string status = timed_out ? "timeout" : "holding";

  publishStatus(true, status, stats, elapsed);

  std::ostringstream warn_text;
  warn_text << std::fixed << std::setprecision(2)
            << "AMCL scan blocked: status=" << status
            << ", elapsed=" << elapsed << "s"
            << ", valid_ratio=" << stats.valid_ratio
            << ", near_ratio=" << stats.near_ratio;
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

void AmclGuard::endHold(const std::string & reason)
{
  if (hold_start_time_) {
    publishStatus(false, reason, std::nullopt, 0.0);
  }
  hold_start_time_.reset();
  hold_allowed_ = false;
}

void AmclGuard::publishStatus(
  const bool active,
  const std::string & reason,
  const std::optional<ScanStats> & stats,
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
