#ifndef AMCL_GUARD__AMCL_GUARD_HPP_
#define AMCL_GUARD__AMCL_GUARD_HPP_

#include "amcl_guard/laser_scan_stabilizer.hpp"
#include "logger/logger.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstddef>
#include <deque>
#include <memory>
#include <optional>
#include <string>

namespace amcl_guard
{

class AmclGuard : public rclcpp::Node
{
public:
  explicit AmclGuard(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct ScanStats
  {
    double valid_ratio = 0.0;
    double near_ratio = 1.0;
    size_t valid = 0;
    size_t near = 0;
    size_t total = 0;
  };

  struct RoughnessStats
  {
    double tilt_deg = 0.0;
    double gyro_xy_rad_s = 0.0;
    double accel_norm_deviation_m_s2 = 0.0;
    double tilt_rate_deg_s = 0.0;
    double accel_norm_rate_m_s3 = 0.0;
    double score = 0.0;
    double age_s = 0.0;
    bool active = false;
    bool hard = false;
    bool hold_active = false;
  };

  struct OdomPose
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  struct OdomHistorySample
  {
    rclcpp::Time stamp;
    OdomPose pose;
    std::optional<double> travel_heading_rad;
  };

  struct ImuSample
  {
    sensor_msgs::msg::Imu msg;
    rclcpp::Time stamp;
    double tilt_deg = 0.0;
    double gyro_xy_rad_s = 0.0;
    double accel_norm_deviation_m_s2 = 0.0;
    double tilt_rate_deg_s = 0.0;
    double accel_norm_rate_m_s3 = 0.0;
  };

  enum class RoughnessThrottleDecision
  {
    kAllowed,
    kDropped,
    kBridgeAllowed,
  };

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  ScanStats scanStats(const sensor_msgs::msg::LaserScan & msg) const;
  std::optional<RoughnessStats> roughnessStats(const sensor_msgs::msg::LaserScan & msg);
  std::optional<ImuSample> nearestImuSample(const rclcpp::Time & scan_time) const;
  double roughnessScore(
    double tilt_deg,
    double gyro_xy_rad_s,
    double accel_norm_deviation_m_s2,
    double tilt_rate_deg_s,
    double accel_norm_rate_m_s3) const;
  double imuTiltDeg(const sensor_msgs::msg::Imu & msg) const;
  rclcpp::Time messageTime(const builtin_interfaces::msg::Time & stamp);
  bool hasGoodPose(const rclcpp::Time & now);
  bool canUseStalePose(const rclcpp::Time & now, double pose_age);
  std::optional<OdomPose> lookupOdomPose(const rclcpp::Time & stamp) const;
  std::optional<OdomPose> lookupLatestOdomPose() const;
  OdomPose transformToOdomPose(
    const geometry_msgs::msg::TransformStamped & transform) const;
  double normalizeAngle(double angle) const;
  std::optional<double> adaptiveTravelHeading(
    const rclcpp::Time & stamp,
    const OdomPose & pose) const;
  bool adaptiveReturnRevisitRisk(const rclcpp::Time & stamp, const OdomPose & pose) const;
  void rememberAdaptiveOdomPose(const rclcpp::Time & stamp, const OdomPose & pose);
  RoughnessThrottleDecision roughnessThrottleDecision(
    const rclcpp::Time & scan_time,
    const std::optional<RoughnessStats> & roughness);
  void resetRoughnessThrottleState();
  void rememberStableScan(
    const sensor_msgs::msg::LaserScan & scan,
    const rclcpp::Time & scan_time,
    const ScanStats & stats,
    const std::optional<RoughnessStats> & roughness);
  std::optional<StabilizedScan> makeStabilizedScan(
    const sensor_msgs::msg::LaserScan & current_scan,
    const rclcpp::Time & scan_time,
    std::string & failure_reason);
  void resetStabilizeEvent();
  void endHold(const std::string & reason);
  void publishStatus(
    bool active,
    const std::string & reason,
    const std::optional<ScanStats> & stats,
    const std::optional<RoughnessStats> & roughness,
    double elapsed);
  void logStatusChange(bool active, const std::string & reason, const std::string & status_text);
  void throttledWarn(const rclcpp::Time & now, const std::string & text);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool enabled_;
  std::string input_scan_topic_;
  std::string output_scan_topic_;
  std::string imu_topic_;
  std::string amcl_pose_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool require_good_pose_;
  double max_pose_age_;
  bool allow_stale_pose_when_stationary_;
  double stale_pose_max_age_;
  double stale_pose_max_odom_translation_;
  double stale_pose_max_odom_rotation_;
  double tf_lookup_timeout_;
  double max_xy_covariance_;
  double max_yaw_covariance_;
  double min_valid_beam_ratio_;
  double max_near_beam_ratio_;
  double near_range_;
  double valid_min_range_;
  double valid_max_range_;
  double max_hold_duration_;
  bool drop_scans_after_timeout_;
  double log_throttle_sec_;
  bool publish_status_topics_;
  bool log_status_changes_;
  bool log_blocked_scan_warnings_;
  std::string log_level_;
  bool enable_imu_roughness_gate_;
  double tilt_threshold_deg_;
  double gyro_xy_threshold_rad_s_;
  double accel_norm_deviation_threshold_m_s2_;
  double tilt_rate_threshold_deg_s_;
  double accel_norm_rate_threshold_m_s3_;
  double max_scan_imu_dt_s_;
  double imu_roughness_hold_duration_s_;
  double imu_roughness_hard_hold_duration_s_;
  double roughness_score_gate_;
  double roughness_score_hard_gate_;
  std::string roughness_gate_mode_;
  double roughness_throttle_min_publish_period_s_;
  bool roughness_throttle_drop_hard_;
  double roughness_max_continuous_drop_duration_s_;
  bool roughness_adaptive_require_return_revisit_;
  double roughness_adaptive_revisit_radius_m_;
  double roughness_adaptive_min_separation_s_;
  double roughness_adaptive_min_heading_delta_rad_;
  double roughness_adaptive_history_duration_s_;
  bool roughness_adaptive_use_travel_heading_;
  double roughness_adaptive_min_travel_distance_m_;
  double roughness_adaptive_travel_heading_window_s_;
  double roughness_stabilize_max_source_age_s_;
  double roughness_stabilize_max_event_duration_s_;
  double roughness_stabilize_min_publish_period_s_;
  double roughness_stabilize_max_translation_m_;
  double roughness_stabilize_max_yaw_rad_;
  double roughness_stabilize_max_roll_pitch_rad_;
  double roughness_stabilize_min_cached_valid_ratio_;
  double roughness_stabilize_min_output_valid_ratio_;
  std::size_t roughness_stabilize_min_output_valid_beams_;
  std::string roughness_stabilize_fallback_mode_;
  bool roughness_stabilize_require_return_revisit_;
  bool use_nearest_imu_sample_;
  double imu_buffer_duration_s_;

  std::optional<sensor_msgs::msg::Imu> last_imu_;
  std::optional<rclcpp::Time> last_imu_time_;
  std::deque<ImuSample> imu_samples_;
  double last_imu_tilt_deg_;
  double last_imu_accel_norm_deviation_m_s2_;
  double last_imu_tilt_rate_deg_s_;
  double last_imu_accel_norm_rate_m_s3_;
  std::optional<rclcpp::Time> last_rough_scan_time_;
  std::optional<rclcpp::Time> last_roughness_publish_time_;
  std::optional<rclcpp::Time> roughness_drop_start_time_;
  bool roughness_throttle_event_active_;
  double last_rough_hold_duration_s_;
  bool last_rough_hold_hard_;
  std::deque<OdomHistorySample> adaptive_odom_history_;
  std::optional<sensor_msgs::msg::LaserScan> last_stable_scan_;
  std::optional<rclcpp::Time> last_stable_scan_time_;
  std::optional<rclcpp::Time> stabilize_event_start_time_;
  std::optional<rclcpp::Time> last_stabilized_publish_time_;
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> last_pose_;
  std::optional<OdomPose> last_pose_odom_pose_;
  std::optional<rclcpp::Time> hold_start_time_;
  bool hold_allowed_;
  std::optional<rclcpp::Time> last_log_time_;
  std::optional<bool> last_logged_status_active_;
  std::string last_logged_status_reason_;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace amcl_guard

#endif  // AMCL_GUARD__AMCL_GUARD_HPP_
