// Copyright 2026 Gole Robotics
#include "slam_3d_localization/slam_3d_localizer.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace
{

Eigen::Isometry3d transformToIsometry(const geometry_msgs::msg::Transform & transform)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond rotation(
    transform.rotation.w, transform.rotation.x, transform.rotation.y,
    transform.rotation.z);
  if (rotation.norm() < 1.0e-9) {
    throw std::runtime_error("TF contains a zero-length quaternion");
  }
  result.linear() = rotation.normalized().toRotationMatrix();
  result.translation() =
    Eigen::Vector3d(transform.translation.x, transform.translation.y, transform.translation.z);
  return result;
}

geometry_msgs::msg::Transform isometryToTransform(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Transform message;
  const Eigen::Quaterniond rotation(transform.linear());
  message.translation.x = transform.translation().x();
  message.translation.y = transform.translation().y();
  message.translation.z = transform.translation().z();
  message.rotation.x = rotation.x();
  message.rotation.y = rotation.y();
  message.rotation.z = rotation.z();
  message.rotation.w = rotation.w();
  return message;
}

diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

}  // namespace

namespace gole::slam_3d_localization
{

Slam3dLocalizer::Slam3dLocalizer(const rclcpp::NodeOptions & options)
: Node("slam_3d_localizer", options)
{
  map_path_ = declare_parameter<std::string>("map_path", "");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  cloud_topic_ = declare_parameter<std::string>(
    "cloud_topic", "/slam_3d/points/center");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
  initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
  pose_topic_ = declare_parameter<std::string>(
    "pose_topic", "/localization_3d/odom");
  aligned_cloud_topic_ = declare_parameter<std::string>(
    "aligned_cloud_topic", "/localization_3d/aligned_points");
  local_map_topic_ = declare_parameter<std::string>(
    "local_map_topic", "/localization_3d/local_map");
  diagnostics_topic_ = declare_parameter<std::string>(
    "diagnostics_topic", "/localization_3d/diagnostics");
  registration_method_ = declare_parameter<std::string>(
    "registration_method", registration_method_);
  cloud_subscription_reliable_ = declare_parameter<bool>(
    "cloud_subscription_reliable", cloud_subscription_reliable_);
  cloud_subscription_depth_ = declare_parameter<int>(
    "cloud_subscription_depth", cloud_subscription_depth_);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  publish_local_map_ = declare_parameter<bool>("publish_local_map", true);
  input_scan_preprocessed_ = declare_parameter<bool>(
    "input_scan_preprocessed", input_scan_preprocessed_);
  map_already_voxelized_ = declare_parameter<bool>(
    "map_already_voxelized", map_already_voxelized_);
  crop_box_enabled_ = declare_parameter<bool>(
    "crop_box_enabled", crop_box_enabled_);
  tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", tf_timeout_s_);
  scan_voxel_leaf_m_ =
    declare_parameter<double>("scan_voxel_leaf_m", scan_voxel_leaf_m_);
  map_voxel_leaf_m_ = declare_parameter<double>("map_voxel_leaf_m", map_voxel_leaf_m_);
  local_map_radius_m_ =
    declare_parameter<double>("local_map_radius_m", local_map_radius_m_);
  min_range_m_ = declare_parameter<double>("min_range_m", min_range_m_);
  max_range_m_ = declare_parameter<double>("max_range_m", max_range_m_);
  z_min_m_ = declare_parameter<double>("z_min_m", z_min_m_);
  z_max_m_ = declare_parameter<double>("z_max_m", z_max_m_);
  crop_min_x_ = declare_parameter<double>("crop_min_x", crop_min_x_);
  crop_max_x_ = declare_parameter<double>("crop_max_x", crop_max_x_);
  crop_min_y_ = declare_parameter<double>("crop_min_y", crop_min_y_);
  crop_max_y_ = declare_parameter<double>("crop_max_y", crop_max_y_);
  crop_min_z_ = declare_parameter<double>("crop_min_z", crop_min_z_);
  crop_max_z_ = declare_parameter<double>("crop_max_z", crop_max_z_);
  max_iterations_ = declare_parameter<int>("max_iterations", max_iterations_);
  max_correspondence_m_ =
    declare_parameter<double>("max_correspondence_m", max_correspondence_m_);
  transformation_epsilon_ =
    declare_parameter<double>("transformation_epsilon", transformation_epsilon_);
  fitness_epsilon_ = declare_parameter<double>("fitness_epsilon", fitness_epsilon_);
  fitness_threshold_ =
    declare_parameter<double>("fitness_threshold", fitness_threshold_);
  min_registration_inlier_ratio_ = declare_parameter<double>(
    "min_registration_inlier_ratio", min_registration_inlier_ratio_);
  max_registration_inlier_rmse_m_ = declare_parameter<double>(
    "max_registration_inlier_rmse_m", max_registration_inlier_rmse_m_);
  max_correction_translation_m_ = declare_parameter<double>(
    "max_correction_translation_m", max_correction_translation_m_);
  max_correction_rotation_deg_ = declare_parameter<double>(
    "max_correction_rotation_deg", max_correction_rotation_deg_);
  update_min_translation_m_ = declare_parameter<double>(
    "update_min_translation_m", update_min_translation_m_);
  update_min_rotation_deg_ = declare_parameter<double>(
    "update_min_rotation_deg", update_min_rotation_deg_);
  use_constant_velocity_prediction_ = declare_parameter<bool>(
    "use_constant_velocity_prediction", use_constant_velocity_prediction_);
  constant_velocity_translation_limit_m_ = declare_parameter<double>(
    "constant_velocity_translation_limit_m", constant_velocity_translation_limit_m_);
  constant_velocity_rotation_limit_deg_ = declare_parameter<double>(
    "constant_velocity_rotation_limit_deg", constant_velocity_rotation_limit_deg_);
  constant_velocity_max_gap_s_ = declare_parameter<double>(
    "constant_velocity_max_gap_s", constant_velocity_max_gap_s_);
  max_cloud_age_s_ = declare_parameter<double>("max_cloud_age_s", max_cloud_age_s_);
  cloud_timestamp_reset_threshold_s_ = declare_parameter<double>(
    "cloud_timestamp_reset_threshold_s", cloud_timestamp_reset_threshold_s_);
  odom_buffer_duration_s_ = declare_parameter<double>(
    "odom_buffer_duration_s", odom_buffer_duration_s_);
  odom_max_interpolation_gap_s_ = declare_parameter<double>(
    "odom_max_interpolation_gap_s", odom_max_interpolation_gap_s_);
  odom_max_nearest_age_s_ = declare_parameter<double>(
    "odom_max_nearest_age_s", odom_max_nearest_age_s_);
  odom_timestamp_reset_threshold_s_ = declare_parameter<double>(
    "odom_timestamp_reset_threshold_s", odom_timestamp_reset_threshold_s_);
  min_scan_points_ = declare_parameter<int>("min_scan_points", min_scan_points_);
  min_local_map_points_ =
    declare_parameter<int>("min_local_map_points", min_local_map_points_);

  const auto declare_positive_size = [this](
    const std::string & name, std::size_t default_value)
    {
      const int value = declare_parameter<int>(name, static_cast<int>(default_value));
      if (value <= 0) {
        throw std::invalid_argument(name + " must be positive");
      }
      return static_cast<std::size_t>(value);
    };
  surfel_config_.voxel_size = declare_parameter<double>(
    "surfel_voxel_size_m", surfel_config_.voxel_size);
  surfel_config_.min_points_per_voxel = declare_positive_size(
    "surfel_min_points_per_voxel", surfel_config_.min_points_per_voxel);
  surfel_config_.max_planarity_ratio = declare_parameter<double>(
    "surfel_max_planarity_ratio", surfel_config_.max_planarity_ratio);
  surfel_config_.min_tangent_variance = declare_parameter<double>(
    "surfel_min_tangent_variance", surfel_config_.min_tangent_variance);
  surfel_config_.max_plane_distance = declare_parameter<double>(
    "surfel_max_plane_distance_m", surfel_config_.max_plane_distance);
  surfel_config_.max_center_distance = declare_parameter<double>(
    "surfel_max_center_distance_m", surfel_config_.max_center_distance);
  surfel_config_.min_correspondences = declare_positive_size(
    "surfel_min_correspondences", surfel_config_.min_correspondences);
  surfel_config_.min_correspondence_ratio = declare_parameter<double>(
    "surfel_min_correspondence_ratio", surfel_config_.min_correspondence_ratio);
  surfel_config_.huber_delta = declare_parameter<double>(
    "surfel_huber_delta_m", surfel_config_.huber_delta);
  surfel_config_.max_iterations = declare_parameter<int>(
    "surfel_max_iterations", surfel_config_.max_iterations);
  surfel_config_.max_backtracking_steps = declare_parameter<int>(
    "surfel_max_backtracking_steps", surfel_config_.max_backtracking_steps);
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

  if (map_path_.empty()) {
    throw std::invalid_argument("map_path must name a readable PCD file");
  }
  const bool valid_crop_bounds =
    std::isfinite(crop_min_x_) && std::isfinite(crop_max_x_) &&
    std::isfinite(crop_min_y_) && std::isfinite(crop_max_y_) &&
    std::isfinite(crop_min_z_) && std::isfinite(crop_max_z_) &&
    crop_min_x_ < crop_max_x_ && crop_min_y_ < crop_max_y_ &&
    crop_min_z_ < crop_max_z_;
  const bool valid_frame_contract =
    !map_frame_.empty() && !odom_frame_.empty() && !base_frame_.empty() &&
    map_frame_ != odom_frame_ && map_frame_ != base_frame_ &&
    odom_frame_ != base_frame_;
  if (!valid_frame_contract ||
    (registration_method_ != "gicp" && registration_method_ != "voxel_surfel") ||
    (input_scan_preprocessed_ && crop_box_enabled_) || !valid_crop_bounds ||
    !::gole::slam_3d_core::VoxelSurfelRegistration::isValidConfig(surfel_config_) ||
    scan_voxel_leaf_m_ <= 0.0 || map_voxel_leaf_m_ <= 0.0 ||
    local_map_radius_m_ <= 0.0 || max_correspondence_m_ <= 0.0 ||
    update_min_translation_m_ < 0.0 || update_min_rotation_deg_ < 0.0 ||
    !std::isfinite(constant_velocity_translation_limit_m_) ||
    constant_velocity_translation_limit_m_ <= 0.0 ||
    !std::isfinite(constant_velocity_rotation_limit_deg_) ||
    constant_velocity_rotation_limit_deg_ <= 0.0 ||
    constant_velocity_rotation_limit_deg_ >= 180.0 ||
    !std::isfinite(constant_velocity_max_gap_s_) ||
    constant_velocity_max_gap_s_ <= 0.0 ||
    !std::isfinite(max_cloud_age_s_) || max_cloud_age_s_ < 0.0 ||
    !std::isfinite(cloud_timestamp_reset_threshold_s_) ||
    cloud_timestamp_reset_threshold_s_ <= 0.0 ||
    cloud_subscription_depth_ <= 0 || odom_buffer_duration_s_ <= 0.0 ||
    odom_max_interpolation_gap_s_ <= 0.0 || odom_max_nearest_age_s_ < 0.0 ||
    odom_max_interpolation_gap_s_ > odom_buffer_duration_s_ ||
    odom_max_nearest_age_s_ > odom_buffer_duration_s_ ||
    odom_timestamp_reset_threshold_s_ <= 0.0 ||
    !std::isfinite(odom_buffer_duration_s_) ||
    !std::isfinite(odom_max_interpolation_gap_s_) ||
    !std::isfinite(odom_max_nearest_age_s_) ||
    !std::isfinite(odom_timestamp_reset_threshold_s_) ||
    fitness_threshold_ <= 0.0 || min_registration_inlier_ratio_ <= 0.0 ||
    min_registration_inlier_ratio_ > 1.0 || max_registration_inlier_rmse_m_ <= 0.0 ||
    max_correction_translation_m_ <= 0.0 ||
    max_correction_rotation_deg_ <= 0.0 || max_correction_rotation_deg_ >= 180.0 ||
    min_scan_points_ <= 0 || min_local_map_points_ <= 0)
  {
    throw std::invalid_argument(
            "frame, registration, preprocessing, voxel, local-map, correction, "
            "cloud QoS, prediction, or odometry timing parameters are invalid");
  }
  odom_buffer_ = OdomPoseBuffer(
    odom_buffer_duration_s_, odom_max_interpolation_gap_s_,
    odom_max_nearest_age_s_, odom_timestamp_reset_threshold_s_);
  constant_velocity_predictor_ = ConstantVelocityPredictor(
    constant_velocity_translation_limit_m_, constant_velocity_rotation_limit_deg_,
    constant_velocity_max_gap_s_);
  cloud_freshness_guard_ = CloudFreshnessGuard(max_cloud_age_s_);
  loadMap();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(pose_topic_, 10);
  aligned_cloud_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(aligned_cloud_topic_, rclcpp::SensorDataQoS());
  local_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    local_map_topic_, rclcpp::QoS(1).transient_local().reliable());
  diagnostics_pub_ =
    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diagnostics_topic_, 10);

  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initial_pose_topic_, rclcpp::QoS(1).reliable(),
    std::bind(&Slam3dLocalizer::initialPoseCallback, this, std::placeholders::_1));
  odom_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions odom_subscription_options;
  odom_subscription_options.callback_group = odom_callback_group_;
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::QoS(100).reliable(),
    std::bind(&Slam3dLocalizer::odomCallback, this, std::placeholders::_1),
    odom_subscription_options);
  auto cloud_qos = rclcpp::QoS(
    rclcpp::KeepLast(std::max(1, cloud_subscription_depth_))).durability_volatile();
  if (cloud_subscription_reliable_) {
    cloud_qos.reliable();
  } else {
    cloud_qos.best_effort();
  }
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, cloud_qos,
    std::bind(&Slam3dLocalizer::cloudCallback, this, std::placeholders::_1));
  no_motion_update_service_ = create_service<std_srvs::srv::Empty>(
    "~/request_nomotion_update",
    std::bind(
      &Slam3dLocalizer::noMotionUpdateCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "Loaded %zu map points from %s; registration=%s, preprocessed_input=%s, "
    "constant_velocity_prediction=%s; waiting for %s",
    global_map_->size(), map_path_.c_str(), registration_method_.c_str(),
    input_scan_preprocessed_ ? "true" : "false",
    use_constant_velocity_prediction_ ? "true" : "false",
    initial_pose_topic_.c_str());
}

void Slam3dLocalizer::loadMap()
{
  CloudT raw;
  if (pcl::io::loadPCDFile<PointT>(map_path_, raw) < 0) {
    throw std::runtime_error("failed to load PCD map: " + map_path_);
  }
  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(raw, raw, valid_indices);
  global_map_->clear();
  if (map_already_voxelized_) {
    *global_map_ = raw;
  } else {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(map_voxel_leaf_m_, map_voxel_leaf_m_, map_voxel_leaf_m_);
    voxel.setInputCloud(raw.makeShared());
    voxel.filter(*global_map_);
  }
  if (global_map_->size() < static_cast<std::size_t>(min_local_map_points_)) {
    throw std::runtime_error("PCD map has too few finite points after downsampling");
  }
}

void Slam3dLocalizer::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
  if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
    RCLCPP_ERROR(
      get_logger(), "Rejected initial pose in frame '%s'; expected '%s'",
      message->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  if (!poseToIsometry(message->pose.pose, pose)) {
    RCLCPP_ERROR(get_logger(), "Rejected initial pose with invalid values or quaternion");
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  {
    std::scoped_lock temporal_lock(odom_buffer_mutex_, tf_cache_mutex_);
    odom_buffer_.clear();
    map_odom_transform_cache_.clearCorrectionPreserveWatermark();
  }
  map_T_base_ = pose;
  initialized_ = true;
  have_processed_odom_ = false;
  have_accepted_odom_ = false;
  have_last_cloud_stamp_ = false;
  constant_velocity_predictor_.reset();
  initial_pose_update_pending_ = true;
  nomotion_update_pending_ = false;
  RCLCPP_INFO(
    get_logger(), "Initial pose set to [%.3f, %.3f, %.3f]",
    pose.translation().x(), pose.translation().y(), pose.translation().z());
}

void Slam3dLocalizer::odomCallback(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (message->header.frame_id != odom_frame_ ||
    message->child_frame_id != base_frame_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring odometry with frames '%s' -> '%s'; expected '%s' -> '%s'",
      message->header.frame_id.c_str(), message->child_frame_id.c_str(),
      odom_frame_.c_str(), base_frame_.c_str());
    return;
  }
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  if (!poseToIsometry(message->pose.pose, pose)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Ignoring odometry with invalid values or quaternion");
    return;
  }
  const rclcpp::Time stamp(message->header.stamp);
  bool timestamp_reset = false;
  {
    std::lock_guard<std::mutex> odom_lock(odom_buffer_mutex_);
    timestamp_reset = odom_buffer_.insert(stamp.nanoseconds(), pose);
  }
  if (timestamp_reset) {
    // Invalidate immediately so no correction from the old odometry epoch can
    // be emitted while a long registration still owns state_mutex_.
    {
      std::lock_guard<std::mutex> tf_lock(tf_cache_mutex_);
      map_odom_transform_cache_.resetOdomEpoch();
    }
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      have_accepted_odom_ = false;
      have_processed_odom_ = false;
      have_last_cloud_stamp_ = false;
      constant_velocity_predictor_.reset();
      // A cloud callback that was already registering may have reanchored
      // after the immediate reset above. Clear that stale correction again,
      // while preserving the newly reset watermark.
      std::lock_guard<std::mutex> tf_lock(tf_cache_mutex_);
      map_odom_transform_cache_.clearCorrectionPreserveWatermark();
      ++odom_timestamp_reset_count_;
    }
    RCLCPP_WARN(
      get_logger(),
      "Detected odometry timestamp rewind; cleared temporal anchors and map-to-odom TF");
    return;
  }
  if (publish_tf_) {
    // Hold the cache lock through sendTransform so correction invalidation and
    // TF publication have a single, deterministic ordering.
    std::lock_guard<std::mutex> tf_lock(tf_cache_mutex_);
    const auto map_T_odom =
      map_odom_transform_cache_.takeForOdomStamp(stamp.nanoseconds());
    if (map_T_odom.has_value()) {
      publishMapToOdomTransform(*map_T_odom, stamp);
    }
  }
}

void Slam3dLocalizer::noMotionUpdateCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;
  std::lock_guard<std::mutex> lock(state_mutex_);
  nomotion_update_pending_ = true;
  RCLCPP_INFO(get_logger(), "No-motion update requested; the next valid cloud will be processed");
}

Slam3dLocalizer::PreparedScan Slam3dLocalizer::prepareScan(
  const sensor_msgs::msg::PointCloud2 & message)
{
  PreparedScan prepared;
  prepared.preprocessed = input_scan_preprocessed_;
  CloudT::Ptr sensor_cloud(new CloudT);
  pcl::fromROSMsg(message, *sensor_cloud);
  prepared.input_points = sensor_cloud->size();
  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(*sensor_cloud, *sensor_cloud, valid_indices);

  if (input_scan_preprocessed_) {
    if (message.header.frame_id != base_frame_) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected preprocessed cloud in frame '%s'; expected exact base frame '%s'",
        message.header.frame_id.c_str(), base_frame_.c_str());
      return prepared;
    }
    prepared.cloud = sensor_cloud;
    prepared.effective_voxel_leaf_m = 0.0;
    return prepared;
  }

  Eigen::Isometry3d base_T_sensor = Eigen::Isometry3d::Identity();
  if (!message.header.frame_id.empty() && message.header.frame_id != base_frame_) {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_, message.header.frame_id, message.header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
      base_T_sensor = transformToIsometry(transform.transform);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(get_logger(), "Cloud TF lookup failed: %s", error.what());
      return prepared;
    }
  }

  CloudT::Ptr base_cloud(new CloudT);
  pcl::transformPointCloud(*sensor_cloud, *base_cloud, base_T_sensor.matrix().cast<float>());
  CloudT::Ptr cropped(new CloudT);
  cropped->reserve(base_cloud->size());
  for (const auto & point : *base_cloud) {
    const double range = std::hypot(point.x, point.y);
    if (range >= min_range_m_ && range <= max_range_m_ &&
      point.z >= z_min_m_ && point.z <= z_max_m_)
    {
      if (crop_box_enabled_ &&
        point.x >= crop_min_x_ && point.x <= crop_max_x_ &&
        point.y >= crop_min_y_ && point.y <= crop_max_y_ &&
        point.z >= crop_min_z_ && point.z <= crop_max_z_)
      {
        ++prepared.robot_crop_removed_points;
        continue;
      }
      cropped->push_back(point);
    }
  }
  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(scan_voxel_leaf_m_, scan_voxel_leaf_m_, scan_voxel_leaf_m_);
  voxel.setInputCloud(cropped);
  CloudT::Ptr filtered(new CloudT);
  voxel.filter(*filtered);
  prepared.cloud = filtered;
  prepared.effective_voxel_leaf_m = scan_voxel_leaf_m_;
  return prepared;
}

Slam3dLocalizer::CloudT::Ptr Slam3dLocalizer::buildLocalMap(
  const Eigen::Vector3d & center) const
{
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(global_map_);
  crop.setMin(
    Eigen::Vector4f(
      center.x() - local_map_radius_m_, center.y() - local_map_radius_m_,
      center.z() - local_map_radius_m_, 1.0));
  crop.setMax(
    Eigen::Vector4f(
      center.x() + local_map_radius_m_, center.y() + local_map_radius_m_,
      center.z() + local_map_radius_m_, 1.0));
  CloudT::Ptr result(new CloudT);
  crop.filter(*result);
  return result;
}

Slam3dLocalizer::RegistrationOutcome Slam3dLocalizer::registerScanToMap(
  const CloudT & scan, const CloudT & local_map,
  const Eigen::Isometry3d & prediction) const
{
  RegistrationOutcome outcome;
  outcome.map_T_base = prediction;
  const auto registration_start = std::chrono::steady_clock::now();
  const auto evaluate_overlap = [&scan, &local_map](
    RegistrationOutcome & result, double max_correspondence_m)
    {
      const Eigen::Matrix3d linear = result.map_T_base.linear();
      result.transform_determinant = linear.determinant();
      result.transform_orthogonality_error =
        (linear.transpose() * linear - Eigen::Matrix3d::Identity()).norm();
      result.inlier_count = 0;
      result.inlier_ratio = 0.0;
      result.inlier_rmse_m = std::numeric_limits<double>::infinity();
      if (!result.map_T_base.matrix().allFinite() || max_correspondence_m <= 0.0) {
        return;
      }

      CloudT aligned;
      pcl::transformPointCloud(
        scan, aligned, result.map_T_base.matrix().cast<float>());
      pcl::KdTreeFLANN<PointT> tree;
      tree.setInputCloud(local_map.makeShared());
      const double max_distance_sq = max_correspondence_m * max_correspondence_m;
      int valid_points = 0;
      double squared_error_sum = 0.0;
      std::vector<int> indices(1);
      std::vector<float> squared_distances(1);
      for (const auto & point : aligned.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z))
        {
          continue;
        }
        ++valid_points;
        if (tree.nearestKSearch(point, 1, indices, squared_distances) <= 0 ||
          squared_distances.front() > max_distance_sq)
        {
          continue;
        }
        ++result.inlier_count;
        squared_error_sum += squared_distances.front();
      }
      if (valid_points > 0) {
        result.inlier_ratio =
          static_cast<double>(result.inlier_count) / static_cast<double>(valid_points);
      }
      if (result.inlier_count > 0) {
        result.inlier_rmse_m = std::sqrt(
          squared_error_sum / static_cast<double>(result.inlier_count));
      }
    };

  if (registration_method_ == "voxel_surfel") {
    const ::gole::slam_3d_core::VoxelSurfelRegistration registration(surfel_config_);
    const auto result = registration.align(scan, local_map, prediction);
    outcome.map_T_base = result.target_T_source;
    outcome.backend_accepted = result.accepted;
    outcome.backend_converged = result.converged;
    outcome.backend_reason = result.reason;
    outcome.fitness = std::isfinite(result.rmse) ?
      result.rmse * result.rmse : std::numeric_limits<double>::infinity();
    outcome.target_voxels = result.target_voxels;
    outcome.planar_surfels = result.planar_surfels;
    outcome.correspondences = result.correspondences;
    outcome.correspondence_ratio = result.correspondence_ratio;
    outcome.surfel_rmse_m = result.rmse;
    outcome.surfel_p95_m = result.p95_abs_residual;
    outcome.observable_rank = result.observable_rank;
    outcome.iterations = result.iterations;
    outcome.backend_runtime_ms = result.total_time_ms;
    evaluate_overlap(outcome, std::max(1.0e-3, surfel_config_.max_center_distance));
    return outcome;
  }

  pcl::GeneralizedIterativeClosestPoint<PointT, PointT> registration;
  registration.setInputSource(scan.makeShared());
  registration.setInputTarget(local_map.makeShared());
  registration.setMaximumIterations(max_iterations_);
  registration.setMaxCorrespondenceDistance(max_correspondence_m_);
  registration.setTransformationEpsilon(transformation_epsilon_);
  registration.setEuclideanFitnessEpsilon(fitness_epsilon_);
  CloudT aligned;
  registration.align(aligned, prediction.matrix().cast<float>());
  outcome.backend_accepted = registration.hasConverged();
  outcome.backend_converged = registration.hasConverged();
  outcome.backend_reason = registration.hasConverged() ? "converged" : "not_converged";
  outcome.fitness = registration.hasConverged() ?
    registration.getFitnessScore(max_correspondence_m_) :
    std::numeric_limits<double>::infinity();
  outcome.map_T_base.matrix() = registration.getFinalTransformation().cast<double>();
  outcome.backend_runtime_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - registration_start).count();
  return outcome;
}

void Slam3dLocalizer::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
{
  const auto start = std::chrono::steady_clock::now();
  const rclcpp::Time stamp(message->header.stamp);
  const std::int64_t cloud_stamp_ns = stamp.nanoseconds();
  bool cloud_timestamp_reset = false;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    const std::int64_t cloud_reset_threshold_ns = static_cast<std::int64_t>(
      std::llround(cloud_timestamp_reset_threshold_s_ * 1.0e9));
    cloud_timestamp_reset = have_last_cloud_stamp_ &&
      cloud_stamp_ns < last_cloud_stamp_ns_ - cloud_reset_threshold_ns;
    if (cloud_timestamp_reset) {
      // Epoch safety precedes freshness and every other early return. The old
      // correction must not keep publishing when ROS time advances but a new
      // cloud epoch starts with timestamps that are already stale.
      have_processed_odom_ = false;
      have_accepted_odom_ = false;
      {
        std::scoped_lock temporal_lock(odom_buffer_mutex_, tf_cache_mutex_);
        odom_buffer_.clear();
        map_odom_transform_cache_.clearCorrectionPreserveWatermark();
      }
      constant_velocity_predictor_.reset();
      ++cloud_timestamp_reset_count_;
      RCLCPP_WARN(
        get_logger(),
        "Detected cloud timestamp rewind; reset localization temporal anchors");
    }
  }
  const CloudFreshnessResult freshness_before = cloud_freshness_guard_.evaluate(
    get_clock()->now().nanoseconds(), cloud_stamp_ns);
  last_cloud_age_valid_.store(freshness_before.ros_time_started);
  last_cloud_age_s_.store(freshness_before.age_s);
  if (freshness_before.enabled && !freshness_before.ros_time_started) {
    publishDiagnostic(
      stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded",
      "waiting_for_ros_time_before_processing", std::numeric_limits<double>::infinity(),
      0, 0, 0.0);
    return;
  }
  if (freshness_before.stale) {
    ++stale_cloud_before_processing_count_;
    publishDiagnostic(
      stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded",
      "stale_cloud_before_processing", std::numeric_limits<double>::infinity(),
      0, 0, 0.0);
    return;
  }

  const PreparedScan prepared_scan = prepareScan(*message);
  const CloudT::Ptr & scan = prepared_scan.cloud;
  if (scan->size() < static_cast<std::size_t>(min_scan_points_)) {
    publishDiagnostic(
      stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded",
      "insufficient_scan_points", std::numeric_limits<double>::infinity(),
      scan->size(), 0, 0.0);
    return;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!initialized_) {
    publishDiagnostic(
      stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN, "uninitialized",
      "waiting_for_initial_pose", std::numeric_limits<double>::infinity(),
      scan->size(), 0, 0.0);
    return;
  }

  // predict() may rebase its internal history for invalid timing gaps. Keep a
  // snapshot after any deliberate epoch reset so a result that becomes stale
  // during registration cannot otherwise mutate predictor state.
  const ConstantVelocityPredictor constant_velocity_predictor_before =
    constant_velocity_predictor_;
  if (!cloud_timestamp_reset && have_last_cloud_stamp_ &&
    cloud_stamp_ns <= last_cloud_stamp_ns_)
  {
    std::size_t odom_buffer_size = 0;
    {
      std::lock_guard<std::mutex> odom_lock(odom_buffer_mutex_);
      odom_buffer_size = odom_buffer_.size();
    }
    publishDiagnostic(
      stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded",
      "nonmonotonic_cloud_stamp", std::numeric_limits<double>::infinity(),
      scan->size(), 0,
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count(),
      nullptr, odom_buffer_size);
    return;
  }
  OdomLookupResult odom_lookup;
  std::size_t odom_buffer_size = 0;
  {
    std::lock_guard<std::mutex> odom_lock(odom_buffer_mutex_);
    if (!cloud_timestamp_reset) {
      odom_lookup = odom_buffer_.lookup(cloud_stamp_ns);
    }
    odom_buffer_size = odom_buffer_.size();
  }
  const bool have_synced_odom = odom_lookup.valid;
  const bool initial_pose_update = initial_pose_update_pending_;
  const bool nomotion_update = nomotion_update_pending_;
  const bool forced_update =
    initial_pose_update || nomotion_update || cloud_timestamp_reset;
  ConstantVelocityPredictionResult constant_velocity_prediction;
  constant_velocity_prediction.map_T_base = map_T_base_;
  constant_velocity_prediction.history_ready = constant_velocity_predictor_.ready();
  const char * prediction_source = "pose_hold";
  if (have_synced_odom && have_processed_odom_ && !forced_update) {
    prediction_source = "odometry_motion_gate";
    const Eigen::Isometry3d motion =
      processed_odom_T_base_.inverse() * odom_lookup.odom_T_base;
    const double translation = motion.translation().norm();
    const double rotation_deg = rotationAngle(motion.linear()) * 180.0 / M_PI;
    if (translation < update_min_translation_m_ &&
      rotation_deg < update_min_rotation_deg_)
    {
      have_last_cloud_stamp_ = true;
      last_cloud_stamp_ns_ = cloud_stamp_ns;
      publishDiagnostic(
        stamp, diagnostic_msgs::msg::DiagnosticStatus::OK, "idle",
        "motion_below_update_threshold", 0.0, scan->size(), 0,
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count(),
        &odom_lookup, odom_buffer_size, nullptr, 0, 0.0,
        prediction_source, &constant_velocity_prediction);
      return;
    }
  }
  Eigen::Isometry3d prediction = map_T_base_;
  if (have_synced_odom && have_accepted_odom_) {
    prediction =
      map_T_base_ * accepted_odom_T_base_.inverse() * odom_lookup.odom_T_base;
    prediction_source = "odometry";
  } else if (!have_synced_odom && use_constant_velocity_prediction_ && !forced_update) {
    constant_velocity_prediction = constant_velocity_predictor_.predict(
      map_T_base_, cloud_stamp_ns);
    if (constant_velocity_prediction.applied) {
      prediction = constant_velocity_prediction.map_T_base;
      prediction_source = "constant_velocity";
    }
  } else if (!have_synced_odom && forced_update) {
    prediction_source = cloud_timestamp_reset ?
      "timestamp_reset_pose_hold" : "forced_pose_hold";
  }
  CloudT::Ptr local_map = buildLocalMap(prediction.translation());
  if (local_map->size() < static_cast<std::size_t>(min_local_map_points_)) {
    have_last_cloud_stamp_ = true;
    last_cloud_stamp_ns_ = cloud_stamp_ns;
    publishDiagnostic(
      stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR, "lost",
      "insufficient_local_map_points", std::numeric_limits<double>::infinity(),
      scan->size(), local_map->size(), 0.0, &odom_lookup, odom_buffer_size,
      nullptr, 0, 0.0, prediction_source, &constant_velocity_prediction);
    return;
  }

  const RegistrationOutcome registration = registerScanToMap(
    *scan, *local_map, prediction);
  const CloudFreshnessResult freshness_after = cloud_freshness_guard_.evaluate(
    get_clock()->now().nanoseconds(), cloud_stamp_ns);
  last_cloud_age_valid_.store(freshness_after.ros_time_started);
  last_cloud_age_s_.store(freshness_after.age_s);
  if (freshness_after.enabled && !freshness_after.ros_time_started) {
    constant_velocity_predictor_ = constant_velocity_predictor_before;
    publishDiagnostic(
      stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded",
      "waiting_for_ros_time_after_registration", registration.fitness,
      scan->size(), local_map->size(),
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count(),
      &odom_lookup, odom_buffer_size, &registration,
      prepared_scan.robot_crop_removed_points,
      prepared_scan.effective_voxel_leaf_m,
      prediction_source, &constant_velocity_prediction);
    return;
  }
  if (freshness_after.stale) {
    constant_velocity_predictor_ = constant_velocity_predictor_before;
    ++stale_cloud_after_registration_count_;
    publishDiagnostic(
      stamp, diagnostic_msgs::msg::DiagnosticStatus::WARN, "degraded",
      "stale_cloud_after_registration", registration.fitness,
      scan->size(), local_map->size(),
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count(),
      &odom_lookup, odom_buffer_size, &registration,
      prepared_scan.robot_crop_removed_points,
      prepared_scan.effective_voxel_leaf_m,
      prediction_source, &constant_velocity_prediction);
    return;
  }

  have_last_cloud_stamp_ = true;
  last_cloud_stamp_ns_ = cloud_stamp_ns;
  if (have_synced_odom) {
    processed_odom_T_base_ = odom_lookup.odom_T_base;
    have_processed_odom_ = true;
  } else {
    have_processed_odom_ = false;
  }

  const double fitness = registration.fitness;
  const Eigen::Isometry3d & result = registration.map_T_base;
  const Eigen::Isometry3d correction = prediction.inverse() * result;
  const double correction_translation = correction.translation().norm();
  const double correction_rotation_deg =
    rotationAngle(correction.linear()) * 180.0 / M_PI;
  const bool use_mapper_overlap_gates = registration_method_ == "voxel_surfel";
  bool accepted = false;
  std::string gate_reason = "accepted";
  if (!registration.backend_accepted) {
    gate_reason = registration_method_ == "voxel_surfel" ?
      "surfel_" + registration.backend_reason : registration.backend_reason;
  } else if (!result.matrix().allFinite() ||
    (use_mapper_overlap_gates &&
    (std::abs(registration.transform_determinant - 1.0) > 0.05 ||
    registration.transform_orthogonality_error > 0.05)))
  {
    gate_reason = "invalid_transform";
  } else if (!std::isfinite(fitness) || fitness > fitness_threshold_) {
    gate_reason = "fitness";
  } else if (use_mapper_overlap_gates &&
    (registration.inlier_count <= 0 ||
    registration.inlier_ratio < min_registration_inlier_ratio_))
  {
    gate_reason = "inlier_ratio";
  } else if (use_mapper_overlap_gates &&
    (!std::isfinite(registration.inlier_rmse_m) ||
    registration.inlier_rmse_m > max_registration_inlier_rmse_m_))
  {
    gate_reason = "inlier_rmse";
  } else if (!std::isfinite(correction_translation) ||
    !std::isfinite(correction_rotation_deg))
  {
    gate_reason = "invalid_correction";
  } else if (correction_translation > max_correction_translation_m_) {
    gate_reason = "translation_correction";
  } else if (correction_rotation_deg > max_correction_rotation_deg_) {
    gate_reason = "rotation_correction";
  } else {
    accepted = true;
  }
  const auto elapsed = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();

  if (!accepted) {
    const std::string reason = initial_pose_update ? "initial_pose_" + gate_reason :
      (nomotion_update ? "nomotion_" + gate_reason : gate_reason);
    publishDiagnostic(
      stamp, diagnostic_msgs::msg::DiagnosticStatus::ERROR, "lost",
      reason, fitness, scan->size(), local_map->size(), elapsed,
      &odom_lookup, odom_buffer_size, &registration,
      prepared_scan.robot_crop_removed_points,
      prepared_scan.effective_voxel_leaf_m,
      prediction_source, &constant_velocity_prediction);
    return;
  }

  // Keep forced-update requests pending until registration really succeeds.
  // Clearing them before the quality gates meant a rejected first scan could
  // be followed by normal tracking without ever publishing the application
  // ACK expected by initial-pose and AMCL-compatible no-motion callers.
  initial_pose_update_pending_ = false;
  nomotion_update_pending_ = false;

  map_T_base_ = result;
  if (have_synced_odom) {
    accepted_odom_T_base_ = odom_lookup.odom_T_base;
    have_accepted_odom_ = true;
    // Only an accepted scan match paired with synchronized odometry may
    // create a map-to-odom correction. Broadcasting belongs to odomCallback.
    std::lock_guard<std::mutex> tf_lock(tf_cache_mutex_);
    if (!map_odom_transform_cache_.reanchor(
        map_T_base_, odom_lookup.odom_T_base, cloud_stamp_ns))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to cache a finite map-to-odom transform after accepted localization");
    }
  } else {
    have_accepted_odom_ = false;
    // Keep the last correction until another synchronized localization can
    // replace it. This preserves a continuous map -> odom chain at odom rate;
    // the unsynchronized scan is still reported in diagnostics.
  }
  if (use_constant_velocity_prediction_) {
    if (nomotion_update) {
      constant_velocity_predictor_.rebaseAccepted(map_T_base_, cloud_stamp_ns);
    } else {
      constant_velocity_predictor_.recordAccepted(map_T_base_, cloud_stamp_ns);
    }
    constant_velocity_prediction.history_ready = constant_velocity_predictor_.ready();
  }
  publishResult(map_T_base_, *scan, stamp, fitness);
  const std::string accepted_reason = initial_pose_update ?
    "initial_pose_update_accepted" :
    (nomotion_update ? "nomotion_update_accepted" :
    (cloud_timestamp_reset ? "timestamp_reset_update_accepted" :
    (have_synced_odom ? "accepted" : "accepted_without_synced_odom")));
  publishDiagnostic(
    stamp,
    have_synced_odom ? diagnostic_msgs::msg::DiagnosticStatus::OK :
    diagnostic_msgs::msg::DiagnosticStatus::WARN,
    "tracking", accepted_reason, fitness, scan->size(), local_map->size(),
    elapsed, &odom_lookup, odom_buffer_size, &registration,
    prepared_scan.robot_crop_removed_points,
    prepared_scan.effective_voxel_leaf_m,
    prediction_source, &constant_velocity_prediction);
}

void Slam3dLocalizer::publishResult(
  const Eigen::Isometry3d & map_T_base, const CloudT & scan_base,
  const rclcpp::Time & stamp, double fitness)
{
  nav_msgs::msg::Odometry pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = map_frame_;
  pose.child_frame_id = base_frame_;
  pose.pose.pose = isometryToPose(map_T_base);
  const double variance = std::max(1.0e-4, fitness);
  pose.pose.covariance[0] = variance;
  pose.pose.covariance[7] = variance;
  pose.pose.covariance[14] = variance;
  pose.pose.covariance[21] = variance;
  pose.pose.covariance[28] = variance;
  pose.pose.covariance[35] = variance;
  pose_pub_->publish(pose);

  CloudT aligned;
  pcl::transformPointCloud(scan_base, aligned, map_T_base.matrix().cast<float>());
  sensor_msgs::msg::PointCloud2 aligned_message;
  pcl::toROSMsg(aligned, aligned_message);
  aligned_message.header.stamp = stamp;
  aligned_message.header.frame_id = map_frame_;
  aligned_cloud_pub_->publish(aligned_message);

  if (publish_local_map_) {
    const auto local_map = buildLocalMap(map_T_base.translation());
    sensor_msgs::msg::PointCloud2 map_message;
    pcl::toROSMsg(*local_map, map_message);
    map_message.header.stamp = stamp;
    map_message.header.frame_id = map_frame_;
    local_map_pub_->publish(map_message);
  }
}

void Slam3dLocalizer::publishMapToOdomTransform(
  const Eigen::Isometry3d & map_T_odom, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = odom_frame_;
  transform.transform = isometryToTransform(map_T_odom);
  tf_broadcaster_->sendTransform(transform);
}

void Slam3dLocalizer::publishDiagnostic(
  const rclcpp::Time & stamp, int level, const std::string & state,
  const std::string & reason, double fitness, std::size_t scan_points,
  std::size_t map_points, double runtime_ms,
  const OdomLookupResult * odom_lookup, std::size_t odom_buffer_size,
  const RegistrationOutcome * registration,
  std::size_t robot_crop_removed_points,
  double effective_scan_voxel_leaf_m,
  const char * prediction_source,
  const ConstantVelocityPredictionResult * constant_velocity_prediction)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = static_cast<uint8_t>(level);
  status.name = get_fully_qualified_name() + std::string(": scan_to_prior_map");
  status.hardware_id = "3d_lidar_localization";
  status.message = state + ": " + reason;
  status.values.push_back(keyValue("state", state));
  status.values.push_back(keyValue("reason", reason));
  status.values.push_back(keyValue("fitness", std::to_string(fitness)));
  status.values.push_back(keyValue("scan_points", std::to_string(scan_points)));
  status.values.push_back(keyValue("local_map_points", std::to_string(map_points)));
  status.values.push_back(keyValue("runtime_ms", std::to_string(runtime_ms)));
  status.values.push_back(keyValue("registration_method", registration_method_));
  status.values.push_back(keyValue("prediction_source", prediction_source));
  status.values.push_back(
    keyValue(
      "constant_velocity_enabled",
      use_constant_velocity_prediction_ ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "constant_velocity_ready",
      constant_velocity_prediction != nullptr ?
      (constant_velocity_prediction->history_ready ? "true" : "false") :
      "not_checked"));
  status.values.push_back(
    keyValue(
      "constant_velocity_status",
      constant_velocity_prediction != nullptr ?
      ConstantVelocityPredictor::statusName(constant_velocity_prediction->status) :
      "not_checked"));
  status.values.push_back(
    keyValue(
      "constant_velocity_applied",
      constant_velocity_prediction != nullptr &&
      constant_velocity_prediction->applied ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "constant_velocity_history_gap_s",
      std::to_string(
        constant_velocity_prediction != nullptr ?
        constant_velocity_prediction->history_gap_s : 0.0)));
  status.values.push_back(
    keyValue(
      "constant_velocity_prediction_gap_s",
      std::to_string(
        constant_velocity_prediction != nullptr ?
        constant_velocity_prediction->prediction_gap_s : 0.0)));
  status.values.push_back(
    keyValue(
      "constant_velocity_raw_translation_m",
      std::to_string(
        constant_velocity_prediction != nullptr ?
        constant_velocity_prediction->raw_translation_m : 0.0)));
  status.values.push_back(
    keyValue(
      "constant_velocity_raw_rotation_deg",
      std::to_string(
        constant_velocity_prediction != nullptr ?
        constant_velocity_prediction->raw_rotation_deg : 0.0)));
  status.values.push_back(
    keyValue(
      "constant_velocity_applied_translation_m",
      std::to_string(
        constant_velocity_prediction != nullptr ?
        constant_velocity_prediction->applied_translation_m : 0.0)));
  status.values.push_back(
    keyValue(
      "constant_velocity_applied_rotation_deg",
      std::to_string(
        constant_velocity_prediction != nullptr ?
        constant_velocity_prediction->applied_rotation_deg : 0.0)));
  status.values.push_back(
    keyValue(
      "constant_velocity_translation_clamped",
      constant_velocity_prediction != nullptr &&
      constant_velocity_prediction->translation_clamped ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "constant_velocity_rotation_clamped",
      constant_velocity_prediction != nullptr &&
      constant_velocity_prediction->rotation_clamped ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "constant_velocity_translation_limit_m",
      std::to_string(constant_velocity_translation_limit_m_)));
  status.values.push_back(
    keyValue(
      "constant_velocity_rotation_limit_deg",
      std::to_string(constant_velocity_rotation_limit_deg_)));
  status.values.push_back(
    keyValue(
      "constant_velocity_max_gap_s",
      std::to_string(constant_velocity_max_gap_s_)));
  status.values.push_back(
    keyValue(
      "cloud_timestamp_reset_threshold_s",
      std::to_string(cloud_timestamp_reset_threshold_s_)));
  status.values.push_back(
    keyValue(
      "cloud_timestamp_reset_count",
      std::to_string(cloud_timestamp_reset_count_.load())));
  const std::size_t stale_before = stale_cloud_before_processing_count_.load();
  const std::size_t stale_after = stale_cloud_after_registration_count_.load();
  const char * stale_stage = reason == "stale_cloud_before_processing" ?
    "before_processing" :
    (reason == "stale_cloud_after_registration" ? "after_registration" : "none");
  status.values.push_back(
    keyValue(
      "cloud_freshness_enabled", max_cloud_age_s_ > 0.0 ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "cloud_age_s",
      last_cloud_age_valid_.load() ?
      std::to_string(last_cloud_age_s_.load()) : "ros_time_not_started"));
  status.values.push_back(
    keyValue("max_cloud_age_s", std::to_string(max_cloud_age_s_)));
  status.values.push_back(keyValue("stale_cloud_drop_stage", stale_stage));
  status.values.push_back(
    keyValue("stale_cloud_before_processing_count", std::to_string(stale_before)));
  status.values.push_back(
    keyValue("stale_cloud_after_registration_count", std::to_string(stale_after)));
  status.values.push_back(
    keyValue("stale_cloud_drop_count", std::to_string(stale_before + stale_after)));
  status.values.push_back(
    keyValue(
      "input_scan_preprocessed", input_scan_preprocessed_ ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "map_already_voxelized", map_already_voxelized_ ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "crop_box_enabled", crop_box_enabled_ ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "robot_crop_removed_points", std::to_string(robot_crop_removed_points)));
  status.values.push_back(
    keyValue(
      "effective_scan_voxel_leaf_m", std::to_string(effective_scan_voxel_leaf_m)));
  status.values.push_back(
    keyValue(
      "registration_backend_accepted",
      registration != nullptr && registration->backend_accepted ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "registration_backend_converged",
      registration != nullptr && registration->backend_converged ? "true" : "false"));
  status.values.push_back(
    keyValue(
      "registration_backend_reason",
      registration != nullptr ? registration->backend_reason : "not_run"));
  status.values.push_back(
    keyValue(
      "registration_backend_runtime_ms",
      std::to_string(registration != nullptr ? registration->backend_runtime_ms : 0.0)));
  status.values.push_back(
    keyValue(
      "surfel_target_voxels",
      std::to_string(registration != nullptr ? registration->target_voxels : 0U)));
  status.values.push_back(
    keyValue(
      "surfel_planar_surfels",
      std::to_string(registration != nullptr ? registration->planar_surfels : 0U)));
  status.values.push_back(
    keyValue(
      "surfel_correspondences",
      std::to_string(registration != nullptr ? registration->correspondences : 0U)));
  status.values.push_back(
    keyValue(
      "surfel_correspondence_ratio",
      std::to_string(registration != nullptr ? registration->correspondence_ratio : 0.0)));
  status.values.push_back(
    keyValue(
      "surfel_rmse_m",
      std::to_string(registration != nullptr ? registration->surfel_rmse_m : 0.0)));
  status.values.push_back(
    keyValue(
      "surfel_p95_m",
      std::to_string(registration != nullptr ? registration->surfel_p95_m : 0.0)));
  status.values.push_back(
    keyValue(
      "surfel_observable_rank",
      std::to_string(registration != nullptr ? registration->observable_rank : 0)));
  status.values.push_back(
    keyValue(
      "surfel_iterations",
      std::to_string(registration != nullptr ? registration->iterations : 0)));
  status.values.push_back(
    keyValue(
      "registration_inlier_count",
      std::to_string(registration != nullptr ? registration->inlier_count : 0)));
  status.values.push_back(
    keyValue(
      "registration_inlier_ratio",
      std::to_string(registration != nullptr ? registration->inlier_ratio : 0.0)));
  status.values.push_back(
    keyValue(
      "registration_inlier_rmse_m",
      std::to_string(registration != nullptr ? registration->inlier_rmse_m : 0.0)));
  status.values.push_back(
    keyValue(
      "registration_transform_determinant",
      std::to_string(
        registration != nullptr ? registration->transform_determinant : 1.0)));
  status.values.push_back(
    keyValue(
      "registration_transform_orthogonality_error",
      std::to_string(
        registration != nullptr ? registration->transform_orthogonality_error : 0.0)));
  status.values.push_back(
    keyValue(
      "cloud_subscription_reliability",
      cloud_subscription_reliable_ ? "reliable" : "best_effort"));
  status.values.push_back(
    keyValue("cloud_subscription_depth", std::to_string(cloud_subscription_depth_)));
  status.values.push_back(
    keyValue("odom_buffer_size", std::to_string(odom_buffer_size)));
  status.values.push_back(
    keyValue(
      "odom_timestamp_reset_count",
      std::to_string(odom_timestamp_reset_count_.load())));
  if (odom_lookup == nullptr) {
    status.values.push_back(keyValue("odom_sync_valid", "false"));
    status.values.push_back(keyValue("odom_sync_mode", "not_checked"));
  } else {
    status.values.push_back(
      keyValue("odom_sync_valid", odom_lookup->valid ? "true" : "false"));
    status.values.push_back(
      keyValue("odom_sync_mode", OdomPoseBuffer::modeName(odom_lookup->mode)));
    status.values.push_back(
      keyValue(
        "odom_sync_nearest_age_ms",
        std::to_string(odom_lookup->nearest_age_s * 1.0e3)));
    status.values.push_back(
      keyValue(
        "odom_sync_bracket_gap_ms",
        std::to_string(odom_lookup->bracket_gap_s * 1.0e3)));
  }
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);
}

bool Slam3dLocalizer::poseToIsometry(
  const geometry_msgs::msg::Pose & pose, Eigen::Isometry3d & transform)
{
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !rotation.coeffs().allFinite() ||
    rotation.norm() < 1.0e-9)
  {
    return false;
  }
  transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation.normalized().toRotationMatrix();
  transform.translation() =
    Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform.matrix().allFinite();
}

geometry_msgs::msg::Pose Slam3dLocalizer::isometryToPose(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose result;
  const Eigen::Quaterniond rotation(transform.linear());
  result.position.x = transform.translation().x();
  result.position.y = transform.translation().y();
  result.position.z = transform.translation().z();
  result.orientation.x = rotation.x();
  result.orientation.y = rotation.y();
  result.orientation.z = rotation.z();
  result.orientation.w = rotation.w();
  return result;
}

double Slam3dLocalizer::rotationAngle(const Eigen::Matrix3d & rotation)
{
  const double cosine = std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

}  // namespace gole::slam_3d_localization
