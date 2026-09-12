// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__LIO_ODOMETRY_HPP_
#define SLAM_3D_LOCALIZATION__LIO_ODOMETRY_HPP_

#include "slam_3d_localization/imu_initializer.hpp"
#include "slam_3d_localization/lio_state_estimator.hpp"
#include "slam_3d_core/voxel_surfel_registration.hpp"

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace gole::slam_3d_localization
{

class LioOdometry : public rclcpp::Node
{
public:
  explicit LioOdometry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  friend class LioOdometryTestPeer;
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  struct TimedPose
  {
    std::int64_t stamp_ns{0};
    Eigen::Isometry3d odom_T_base{Eigen::Isometry3d::Identity()};
  };

  struct PendingCloud
  {
    sensor_msgs::msg::PointCloud2::SharedPtr message;
    std::int64_t start_stamp_ns{0};
    std::int64_t end_stamp_ns{0};
    bool has_point_time{false};
  };

  struct LidarKeyframe
  {
    Eigen::Isometry3d odom_T_base{Eigen::Isometry3d::Identity()};
    CloudT::Ptr cloud_odom{new CloudT};
  };

  struct WheelObservation
  {
    std::int64_t stamp_ns{0};
    double base_forward_speed_mps{0.0};
    double standard_deviation_mps{0.0};
  };

  struct RegistrationOutcome
  {
    bool accepted{false};
    bool converged{false};
    std::string reason{"not_run"};
    Eigen::Isometry3d odom_T_base{Eigen::Isometry3d::Identity()};
    double fitness{0.0};
    double rmse_m{0.0};
    double p95_m{0.0};
    double correspondence_ratio{0.0};
    int observable_rank{0};
    Eigen::Vector3d weak_rotation_axes{Eigen::Vector3d::Zero()};
    Eigen::Vector3d weak_translation_axes{Eigen::Vector3d::Zero()};
    double runtime_ms{0.0};
  };

  void declareAndValidateParameters();
  void setupRosInterfaces();
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr message);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message);
  void wheelCallback(const nav_msgs::msg::Odometry::SharedPtr message);
  void processPendingClouds();
  void processCloud(const PendingCloud & pending);
  void processLidarOnlyCloud(
    const PendingCloud & pending, const Eigen::Isometry3d & base_T_sensor);
  void resetForTimeEpoch(const std::string & reason);
  void advanceCorrectedAnchor(
    const LioStateEstimator & estimator_at_stamp, std::int64_t stamp_ns);
  void invalidateOlderPublications(std::uint64_t minimum_revision);
  void boundImuBuffer();

  std::optional<ImuMeasurement> transformImuMeasurement(
    const sensor_msgs::msg::Imu & message) const;
  bool buildImuTrajectory(
    std::int64_t start_stamp_ns, std::int64_t end_stamp_ns,
    LioStateEstimator & estimator_at_end,
    std::vector<TimedPose> & trajectory) const;
  CloudT::Ptr prepareAndDeskewCloud(
    const sensor_msgs::msg::PointCloud2 & message,
    const Eigen::Isometry3d & base_T_sensor,
    const std::vector<TimedPose> & trajectory,
    const Eigen::Isometry3d & odom_T_base_reference,
    std::int64_t start_stamp_ns, std::int64_t end_stamp_ns,
    bool has_point_time,
    std::size_t & valid_input_points) const;
  RegistrationOutcome registerScan(
    const CloudT & scan_base,
    const CloudT & local_map,
    const Eigen::Isometry3d & prediction) const;
  CloudT::Ptr buildLocalSubmap(
    const CloudT::ConstPtr & submap_snapshot, const Eigen::Vector3d & center) const;
  bool updateKeyframeMapIfCurrent(
    const CloudT & scan_base, const Eigen::Isometry3d & odom_T_base,
    std::uint64_t expected_state_revision, bool force_keyframe,
    bool mark_lidar_reference, std::size_t & map_points);
  CloudT::Ptr buildRollingSubmap(
    const std::deque<LidarKeyframe> & keyframes) const;
  bool shouldAddKeyframe(
    const std::deque<LidarKeyframe> & keyframes,
    const Eigen::Isometry3d & pose) const;
  void repropagateLiveState();
  bool applyWheelObservation(
    LioStateEstimator & estimator, const WheelObservation & observation) const;
  Eigen::Isometry3d odomTBase(const LioStateEstimator & estimator) const;
  Eigen::Isometry3d basePoseToBodyPose(const Eigen::Isometry3d & odom_T_base) const;

  nav_msgs::msg::Odometry makeOdometry(
    const LioStateEstimator & estimator, const rclcpp::Time & stamp) const;
  void publishOdometry(
    const LioStateEstimator & estimator, const rclcpp::Time & stamp,
    std::uint64_t publication_revision);
  bool publishScanOutputsIfCurrent(
    const LioStateEstimator & estimator, const CloudT & cloud,
    const rclcpp::Time & stamp, std::uint64_t expected_state_revision);
  void publishDiagnostic(
    const rclcpp::Time & stamp, int level, const std::string & state,
    const std::string & reason, const RegistrationOutcome * registration = nullptr,
    std::size_t scan_points = 0U, std::size_t map_points = 0U);

  static bool pointCloudTimeBounds(
    const sensor_msgs::msg::PointCloud2 & message,
    const std::string & field_name,
    const std::string & mode,
    double max_scan_duration_s,
    double header_tolerance_s,
    bool require_point_time,
    std::int64_t & start_stamp_ns,
    std::int64_t & end_stamp_ns,
    bool & has_point_time);
  static Eigen::Isometry3d interpolatePose(
    const std::vector<TimedPose> & trajectory, std::int64_t stamp_ns);
  static double rotationAngle(const Eigen::Matrix3d & rotation);

  std::string cloud_topic_;
  std::string imu_topic_;
  std::string wheel_topic_;
  std::string odom_topic_;
  std::string scan_odom_topic_;
  std::string deskewed_cloud_topic_;
  std::string diagnostics_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  std::string point_time_field_{"timestamp"};
  std::string point_time_mode_{"auto"};
  std::string registration_method_{"voxel_surfel"};
  bool publish_tf_{true};
  // IMU remains active; bias learning is independently disabled by default.
  bool imu_enabled_{true};
  bool wheel_correction_enabled_{false};
  bool require_point_time_{true};
  bool cloud_subscription_reliable_{false};
  int cloud_subscription_depth_{4};
  int pending_cloud_limit_{4};
  double tf_timeout_s_{0.05};
  double imu_epoch_reset_threshold_s_{1.0};
  double imu_buffer_duration_s_{5.0};
  double max_scan_duration_s_{0.20};
  double point_time_header_tolerance_s_{0.02};
  double imu_gap_position_std_m_{0.20};
  double imu_gap_velocity_std_mps_{0.75};
  double imu_gap_orientation_std_deg_{5.0};
  // Compute-first defaults: coarse clouds and a small rolling submap.
  double scan_voxel_leaf_m_{0.8};
  double submap_voxel_leaf_m_{0.8};
  double local_submap_radius_m_{15.0};
  double min_range_m_{0.5};
  double max_range_m_{30.0};
  double z_min_m_{-5.0};
  double z_max_m_{8.0};
  bool crop_box_enabled_{true};
  Eigen::Vector3d crop_min_{-0.7, -0.55, -0.8};
  Eigen::Vector3d crop_max_{0.7, 0.55, 0.9};
  int min_scan_points_{100};
  int min_submap_points_{100};
  int max_keyframes_{10};
  double keyframe_translation_m_{1.5};
  double keyframe_rotation_deg_{15.0};
  int gicp_max_iterations_{8};
  double gicp_max_correspondence_m_{2.0};
  double fitness_threshold_{1.0};
  double max_correction_translation_m_{1.5};
  double max_correction_rotation_deg_{25.0};
  double lidar_translation_std_min_m_{0.03};
  double lidar_rotation_std_min_deg_{0.5};
  double lidar_noise_scale_{0.25};
  double lidar_pose_nis_gate_{100.0};
  double lidar_weak_axis_noise_multiplier_{10.0};
  double wheel_max_age_s_{0.05};
  double wheel_max_abs_speed_mps_{3.0};
  double wheel_max_innovation_mps_{1.0};
  double wheel_fallback_std_mps_{0.20};
  Eigen::Matrix3d base_R_imu_{Eigen::Matrix3d::Identity()};
  // Translation from base_link origin to the physical IMU origin, expressed
  // in base-aligned axes. The ESKF position is the IMU origin so rotational
  // lever-arm motion is not silently integrated as base translation.
  Eigen::Vector3d base_p_imu_{Eigen::Vector3d::Zero()};
  LioStateEstimator::Config state_config_;
  ImuInitializationConfig initializer_config_;
  ::gole::slam_3d_core::VoxelSurfelRegistration::Config surfel_config_;

  mutable std::mutex state_mutex_;
  mutable std::mutex submap_mutex_;
  StationaryImuInitializer imu_initializer_;
  LioStateEstimator corrected_estimator_;
  LioStateEstimator live_estimator_;
  std::deque<ImuMeasurement> imu_buffer_;
  std::deque<WheelObservation> wheel_observations_;
  std::deque<PendingCloud> pending_clouds_;
  std::deque<LidarKeyframe> keyframes_;
  CloudT::Ptr rolling_submap_{new CloudT};
  bool have_lidar_reference_{false};
  Eigen::Isometry3d lidar_only_pose_{Eigen::Isometry3d::Identity()};
  std::int64_t lidar_only_input_stamp_ns_{-1};
  std::int64_t lidar_only_pose_stamp_ns_{-1};
  bool processing_cloud_{false};
  std::int64_t processing_cloud_end_stamp_ns_{0};
  std::int64_t latest_imu_stamp_ns_{0};
  std::int64_t last_processed_cloud_stamp_ns_{0};
  std::size_t dropped_cloud_count_{0U};
  std::size_t rejected_registration_count_{0U};
  std::size_t accepted_registration_count_{0U};
  std::size_t imu_reset_count_{0U};
  std::size_t imu_gap_recovery_count_{0U};
  std::size_t wheel_update_count_{0U};
  std::uint64_t state_revision_{0U};
  std::uint64_t publication_revision_{0U};
  std::string last_propagation_status_{"not_initialized"};

  std::mutex odom_publish_mutex_;
  bool have_published_odom_{false};
  std::int64_t last_published_odom_stamp_ns_{0};
  std::uint64_t last_published_revision_{0U};
  std::uint64_t minimum_valid_publication_revision_{0U};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr wheel_callback_group_;
  rclcpp::TimerBase::SharedPtr cloud_processing_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr scan_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_cloud_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace gole::slam_3d_localization

#endif  // SLAM_3D_LOCALIZATION__LIO_ODOMETRY_HPP_
