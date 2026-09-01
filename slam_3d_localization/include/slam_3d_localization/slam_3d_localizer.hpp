// Copyright 2026 Gole Robotics
#ifndef SLAM_3D_LOCALIZATION__SLAM_3D_LOCALIZER_HPP_
#define SLAM_3D_LOCALIZATION__SLAM_3D_LOCALIZER_HPP_

#include "slam_3d_localization/cloud_freshness_guard.hpp"
#include "slam_3d_localization/constant_velocity_predictor.hpp"
#include "slam_3d_localization/map_odom_transform_cache.hpp"
#include "slam_3d_localization/odom_pose_buffer.hpp"
#include "slam_3d_core/voxel_surfel_registration.hpp"

#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <mutex>
#include <string>

namespace gole::slam_3d_localization
{

class Slam3dLocalizer : public rclcpp::Node
{
public:
  explicit Slam3dLocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  struct PreparedScan
  {
    CloudT::Ptr cloud{new CloudT};
    std::size_t input_points{0};
    std::size_t robot_crop_removed_points{0};
    double effective_voxel_leaf_m{0.0};
    bool preprocessed{false};
  };

  struct RegistrationOutcome
  {
    Eigen::Isometry3d map_T_base{Eigen::Isometry3d::Identity()};
    bool backend_accepted{false};
    bool backend_converged{false};
    std::string backend_reason{"not_run"};
    double fitness{0.0};
    std::size_t target_voxels{0};
    std::size_t planar_surfels{0};
    std::size_t correspondences{0};
    double correspondence_ratio{0.0};
    double surfel_rmse_m{0.0};
    double surfel_p95_m{0.0};
    int observable_rank{0};
    int iterations{0};
    double backend_runtime_ms{0.0};
    int inlier_count{0};
    double inlier_ratio{0.0};
    double inlier_rmse_m{0.0};
    double transform_determinant{1.0};
    double transform_orthogonality_error{0.0};
  };

  void loadMap();
  void publishGlobalMap();
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr message);
  void noMotionUpdateCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message);
  PreparedScan prepareScan(const sensor_msgs::msg::PointCloud2 & message);
  CloudT::Ptr buildLocalMap(const Eigen::Vector3d & center) const;
  RegistrationOutcome registerScanToMap(
    const CloudT & scan, const CloudT & local_map,
    const Eigen::Isometry3d & prediction) const;
  void publishResult(
    const Eigen::Isometry3d & map_T_base, const CloudT & scan_base,
    const rclcpp::Time & stamp, double fitness);
  void publishMapToOdomTransform(
    const Eigen::Isometry3d & map_T_odom, const rclcpp::Time & stamp);
  void publishDiagnostic(
    const rclcpp::Time & stamp, int level, const std::string & state,
    const std::string & reason, double fitness, std::size_t scan_points,
    std::size_t map_points, double runtime_ms,
    const OdomLookupResult * odom_lookup = nullptr,
    std::size_t odom_buffer_size = 0,
    const RegistrationOutcome * registration = nullptr,
    std::size_t robot_crop_removed_points = 0,
    double effective_scan_voxel_leaf_m = 0.0,
    const char * prediction_source = "not_checked",
    const ConstantVelocityPredictionResult * constant_velocity_prediction = nullptr);

  static bool poseToIsometry(
    const geometry_msgs::msg::Pose & pose, Eigen::Isometry3d & transform);
  static geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d & transform);
  static double rotationAngle(const Eigen::Matrix3d & rotation);

  std::string map_path_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string initial_pose_topic_;
  std::string pose_topic_;
  std::string aligned_cloud_topic_;
  std::string local_map_topic_;
  std::string global_map_topic_;
  std::string diagnostics_topic_;
  std::string registration_method_{"gicp"};
  bool cloud_subscription_reliable_{false};
  int cloud_subscription_depth_{5};
  bool publish_tf_{true};
  bool publish_local_map_{true};
  bool publish_global_map_{true};
  bool input_scan_preprocessed_{false};
  bool map_already_voxelized_{false};
  bool crop_box_enabled_{false};
  double tf_timeout_s_{0.05};
  double scan_voxel_leaf_m_{0.25};
  double map_voxel_leaf_m_{0.35};
  double local_map_radius_m_{35.0};
  double min_range_m_{0.5};
  double max_range_m_{80.0};
  double z_min_m_{-5.0};
  double z_max_m_{8.0};
  double crop_min_x_{-0.7};
  double crop_max_x_{0.7};
  double crop_min_y_{-0.55};
  double crop_max_y_{0.55};
  double crop_min_z_{-0.8};
  double crop_max_z_{0.9};
  int max_iterations_{40};
  double max_correspondence_m_{2.0};
  double transformation_epsilon_{1.0e-3};
  double fitness_epsilon_{1.0e-3};
  double fitness_threshold_{1.0};
  double min_registration_inlier_ratio_{0.15};
  double max_registration_inlier_rmse_m_{1.5};
  double max_correction_translation_m_{2.0};
  double max_correction_rotation_deg_{30.0};
  double update_min_translation_m_{0.10};
  double update_min_rotation_deg_{2.0};
  bool use_constant_velocity_prediction_{false};
  double constant_velocity_translation_limit_m_{0.30};
  double constant_velocity_rotation_limit_deg_{10.0};
  double constant_velocity_max_gap_s_{0.50};
  double max_cloud_age_s_{0.0};
  double cloud_timestamp_reset_threshold_s_{1.0};
  double odom_buffer_duration_s_{5.0};
  double odom_max_interpolation_gap_s_{0.20};
  double odom_max_nearest_age_s_{0.05};
  double odom_timestamp_reset_threshold_s_{1.0};
  int min_scan_points_{100};
  int min_local_map_points_{500};
  ::gole::slam_3d_core::VoxelSurfelRegistration::Config surfel_config_{};

  CloudT::Ptr global_map_{new CloudT};
  pcl::PCLPointCloud2 visualization_map_;
  // Lock order: state_mutex_ may be held before either specialized mutex.
  // The odometry callback never holds a specialized mutex while acquiring
  // state_mutex_, and never holds odom_buffer_mutex_ while acquiring
  // tf_cache_mutex_.
  mutable std::mutex state_mutex_;
  mutable std::mutex odom_buffer_mutex_;
  mutable std::mutex tf_cache_mutex_;
  bool initialized_{false};
  bool have_accepted_odom_{false};
  bool have_processed_odom_{false};
  bool initial_pose_update_pending_{false};
  bool nomotion_update_pending_{false};
  bool have_last_cloud_stamp_{false};
  std::int64_t last_cloud_stamp_ns_{0};
  std::atomic<std::size_t> odom_timestamp_reset_count_{0};
  std::atomic<std::size_t> cloud_timestamp_reset_count_{0};
  std::atomic<std::size_t> stale_cloud_before_processing_count_{0};
  std::atomic<std::size_t> stale_cloud_after_registration_count_{0};
  std::atomic<double> last_cloud_age_s_{0.0};
  std::atomic<bool> last_cloud_age_valid_{false};
  Eigen::Isometry3d map_T_base_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d accepted_odom_T_base_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d processed_odom_T_base_{Eigen::Isometry3d::Identity()};
  MapOdomTransformCache map_odom_transform_cache_;
  OdomPoseBuffer odom_buffer_{
    odom_buffer_duration_s_, odom_max_interpolation_gap_s_,
    odom_max_nearest_age_s_, odom_timestamp_reset_threshold_s_};
  ConstantVelocityPredictor constant_velocity_predictor_{
    constant_velocity_translation_limit_m_, constant_velocity_rotation_limit_deg_,
    constant_velocity_max_gap_s_};
  CloudFreshnessGuard cloud_freshness_guard_{max_cloud_age_s_};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr no_motion_update_service_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
};

}  // namespace gole::slam_3d_localization
#endif  // SLAM_3D_LOCALIZATION__SLAM_3D_LOCALIZER_HPP_
