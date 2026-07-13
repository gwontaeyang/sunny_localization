#ifndef AMCL_GUARD__AMCL_GUARD_HPP_
#define AMCL_GUARD__AMCL_GUARD_HPP_

#include "logger/logger.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cstddef>
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

  struct OdomPose
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  ScanStats scanStats(const sensor_msgs::msg::LaserScan & msg) const;
  bool hasGoodPose(const rclcpp::Time & now);
  bool canUseStalePose(const rclcpp::Time & now, double pose_age);
  std::optional<OdomPose> lookupOdomPose(const rclcpp::Time & stamp) const;
  std::optional<OdomPose> lookupLatestOdomPose() const;
  OdomPose transformToOdomPose(
    const geometry_msgs::msg::TransformStamped & transform) const;
  double normalizeAngle(double angle) const;
  void endHold(const std::string & reason);
  void publishStatus(
    bool active,
    const std::string & reason,
    const std::optional<ScanStats> & stats,
    double elapsed);
  void logStatusChange(bool active, const std::string & reason, const std::string & status_text);
  void throttledWarn(const rclcpp::Time & now, const std::string & text);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool enabled_;
  std::string input_scan_topic_;
  std::string output_scan_topic_;
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
