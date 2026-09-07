#include "ament_index_cpp/get_package_share_directory.hpp"

#include <cmath>
#include <filesystem>
#include <stdexcept>

#include "odom_estimation/odom_estimation.hpp"
#include "odom_estimation/imu_transform.hpp"
#include "odom_estimation/imu_bias_config.hpp"
#include "odom_estimation/odom_estimator_factory.hpp"

namespace gole {
namespace odom_estimation {

OdomEstimation::OdomEstimation()
    : rclcpp_lifecycle::LifecycleNode("odom_estimation_node")
    , using_landmark_correction_(false)
{
    logger_ = gole::logger::Logger::createPackageLogger(this->get_name(), spdlog::level::level_enum::info);
    logger_->info("localization starting...");
    transform_last_log_time_ = std::chrono::steady_clock::now();
}

// -----------------------------------------------------------------------------
// Lifecycle Functions
// -----------------------------------------------------------------------------

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OdomEstimation::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    try {
        logger_->info("Configuring OdomEstimation...");
        setupParameters();
        gole::logger::Logger::setLogLevel(this->get_name(), spdlog::level::from_str(declare_parameter("log_level", "info")));

        // Setup TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Setup publishers and subscribers
        setupPublishers();
        setupSubscribers();

        initialize();

        logger_->info("OdomEstimation configured successfully");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    catch (const std::exception& e) {
        logger_->error("Exception during configuration: {}", e.what());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OdomEstimation::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    logger_->info("Activating OdomEstimation...");

    // Activate publishers
    pose_pub_->on_activate();
    marker_pub_->on_activate();
    landmark_pose_pub_->on_activate();
    gyro_bias_pub_->on_activate();
    
    // Activate diagnostic manager
    if (diagnostic_manager_) {
        diagnostic_manager_->activate();
    }

    // Create and start timer only when activated
    transform_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&OdomEstimation::timerCallback, this)
    );

    logger_->info("OdomEstimation activated successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OdomEstimation::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    logger_->info("Deactivating OdomEstimation...");

    // Stop timers when deactivating
    if (transform_timer_) {
        transform_timer_->cancel();
        transform_timer_.reset();
    }

    // Deactivate publishers
    pose_pub_->on_deactivate();
    marker_pub_->on_deactivate();
    landmark_pose_pub_->on_deactivate();
    gyro_bias_pub_->on_deactivate();
    
    // Deactivate diagnostic manager
    if (diagnostic_manager_) {
        diagnostic_manager_->deactivate();
    }

    logger_->info("OdomEstimation deactivated successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OdomEstimation::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    logger_->info("Cleaning up OdomEstimation...");
    
    // Reset all components
    odom_sub_.reset();
    imu_sub_.reset();
    pose_command_sub_.reset();
    odom_estimator_.reset();

    logger_->info("OdomEstimation cleaned up successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OdomEstimation::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
    logger_->info("Shutting down OdomEstimation...");
    
    // Reset all components
    odom_sub_.reset();
    imu_sub_.reset();
    pose_command_sub_.reset();
    odom_estimator_.reset();

    logger_->info("OdomEstimation shut down successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// -----------------------------------------------------------------------------
// Initialize Functions
// -----------------------------------------------------------------------------

void OdomEstimation::setupParameters()
{
    // Create shared parameter structure
    params_ = std::make_shared<Params>();
    
    std::string odom_estimator_type_str = declare_parameter("odom_estimator_type", std::string("EKF"));
    odom_estimator_type_ = stringToOdomEstimatorType(odom_estimator_type_str);
    logger_->info("OdomEstimation parameter 'odom_estimator_type' set to '{}'", odom_estimator_type_str);
    
    params_->load_initial_gyro_bias = declare_parameter("load_initial_gyro_bias", false);
    params_->gyro_bias_yaml_path = declare_parameter(
        "gyro_bias_yaml_path", std::string("~/.ros/imu_calibration.yaml"));
    params_->allow_initial_gyro_bias_fallback = declare_parameter(
        "allow_initial_gyro_bias_fallback", false);
    params_->gyro_bias_fallback_yaml_path = declare_parameter(
        "gyro_bias_fallback_yaml_path", std::string(""));
    params_->max_initial_gyro_bias_abs = declare_parameter(
        "max_initial_gyro_bias_abs", 0.05);
    params_->imu_target_frame = declare_parameter(
        "imu_target_frame", std::string("base_link"));
    params_->imu_tf_lookup_timeout = declare_parameter("imu_tf_lookup_timeout", 0.05);
    params_->use_wheel_message_covariance = declare_parameter(
        "use_wheel_message_covariance", true);
    params_->wheel_velocity_noise = declare_parameter("wheel_velocity_noise", 0.0016);
    params_->wheel_min_velocity_variance = declare_parameter(
        "wheel_min_velocity_variance", 1.0e-6);
    params_->enable_3d_estimation = declare_parameter(
        "enable_3d_estimation", false);
    params_->use_imu_message_covariance = declare_parameter(
        "use_imu_message_covariance", true);
    params_->gyro_noise = declare_parameter("gyro_noise", 7.615435494667714e-5);
    params_->gyro_min_variance = declare_parameter("gyro_min_variance", 1.0e-8);
    params_->gyro_bias_random_walk = declare_parameter(
        "gyro_bias_random_walk", 7.615435494667715e-9);
    params_->initial_yaw_covariance = declare_parameter(
        "initial_yaw_covariance", 0.030461741978670857);
    params_->initial_gyro_bias_covariance = declare_parameter(
        "initial_gyro_bias_covariance", 7.615435494667714e-5);
    params_->initial_z_covariance = declare_parameter(
        "initial_z_covariance", 0.04);
    params_->initial_roll_pitch_covariance = declare_parameter(
        "initial_roll_pitch_covariance", 0.007615435494667714);
    params_->vertical_motion_noise = declare_parameter(
        "vertical_motion_noise", 0.0025);
    params_->roll_pitch_process_noise = declare_parameter(
        "roll_pitch_process_noise", 7.615435494667714e-5);
    params_->gyro_initial_bias_enabled = declare_parameter(
        "gyro_initial_bias_enabled", true);
    params_->gyro_initial_bias_duration = declare_parameter(
        "gyro_initial_bias_duration", 2.0);
    params_->gyro_initial_bias_min_samples = declare_parameter(
        "gyro_initial_bias_min_samples", 100);
    params_->gyro_stationary_threshold = declare_parameter(
        "gyro_stationary_threshold", 0.03490658503988659);
    params_->wheel_stationary_threshold = declare_parameter(
        "wheel_stationary_threshold", 0.005);
    params_->wheel_stationary_angular_threshold = declare_parameter(
        "wheel_stationary_angular_threshold", 0.001);
    params_->stationary_gyro_bias_variance = declare_parameter(
        "stationary_gyro_bias_variance", 1.2184696791468346e-5);
    params_->max_prediction_dt = declare_parameter("max_prediction_dt", 0.5);
    params_->reject_zero_timestamp = declare_parameter("reject_zero_timestamp", true);
    params_->vy_zero_constraint_enabled = declare_parameter(
        "vy_zero_constraint_enabled", true);
    params_->vy_zero_constraint_noise = declare_parameter(
        "vy_zero_constraint_noise", 0.04);

    const auto require_positive = [](double value, const char* name) {
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(std::string(name) + " must be finite and > 0");
        }
    };
    require_positive(params_->imu_tf_lookup_timeout, "imu_tf_lookup_timeout");
    require_positive(params_->wheel_velocity_noise, "wheel_velocity_noise");
    require_positive(params_->wheel_min_velocity_variance, "wheel_min_velocity_variance");
    require_positive(params_->gyro_noise, "gyro_noise");
    require_positive(params_->gyro_min_variance, "gyro_min_variance");
    require_positive(params_->gyro_bias_random_walk, "gyro_bias_random_walk");
    require_positive(params_->initial_yaw_covariance, "initial_yaw_covariance");
    require_positive(
        params_->initial_gyro_bias_covariance, "initial_gyro_bias_covariance");
    require_positive(params_->initial_z_covariance, "initial_z_covariance");
    require_positive(
        params_->initial_roll_pitch_covariance, "initial_roll_pitch_covariance");
    require_positive(params_->vertical_motion_noise, "vertical_motion_noise");
    require_positive(params_->roll_pitch_process_noise, "roll_pitch_process_noise");
    require_positive(params_->gyro_initial_bias_duration, "gyro_initial_bias_duration");
    require_positive(params_->gyro_stationary_threshold, "gyro_stationary_threshold");
    require_positive(params_->wheel_stationary_threshold, "wheel_stationary_threshold");
    require_positive(
        params_->wheel_stationary_angular_threshold,
        "wheel_stationary_angular_threshold");
    require_positive(
        params_->stationary_gyro_bias_variance, "stationary_gyro_bias_variance");
    require_positive(params_->max_prediction_dt, "max_prediction_dt");
    require_positive(params_->vy_zero_constraint_noise, "vy_zero_constraint_noise");
    if (params_->imu_target_frame.empty()) {
        throw std::invalid_argument("imu_target_frame must not be empty");
    }
    if (params_->gyro_initial_bias_min_samples <= 0) {
        throw std::invalid_argument("gyro_initial_bias_min_samples must be > 0");
    }

    logger_->info(
        "Odometry motion mode: {}",
        params_->enable_3d_estimation
            ? "3D (x/y/z + roll/pitch/yaw)"
            : "planar (x/y/yaw; z/roll/pitch constrained to zero)");

    if (params_->load_initial_gyro_bias) {
        std::string selected_path = params_->gyro_bias_yaml_path;
        const std::string resolved_primary = expandUserPath(selected_path);
        if (!std::filesystem::exists(resolved_primary) &&
            params_->allow_initial_gyro_bias_fallback &&
            !params_->gyro_bias_fallback_yaml_path.empty()) {
            const std::string resolved_fallback = expandUserPath(
                params_->gyro_bias_fallback_yaml_path);
            if (std::filesystem::exists(resolved_fallback)) {
                selected_path = params_->gyro_bias_fallback_yaml_path;
                logger_->warn(
                    "Active gyro bias YAML '{}' is absent; using startup fallback '{}'",
                    resolved_primary,
                    resolved_fallback);
            }
        }
        const auto gyro_bias = loadGyroBiasCalibration(
            selected_path,
            params_->max_initial_gyro_bias_abs);
        params_->initial_gyro_bias_x = gyro_bias[0];
        params_->initial_gyro_bias_y = gyro_bias[1];
        params_->initial_gyro_bias_z = gyro_bias[2];
        logger_->info(
            "Initial gyro bias enabled: [{:.9f}, {:.9f}, {:.9f}] rad/s, source='{}'",
            params_->initial_gyro_bias_x,
            params_->initial_gyro_bias_y,
            params_->initial_gyro_bias_z,
            expandUserPath(selected_path));
    } else {
        logger_->info("Initial gyro bias YAML loading disabled; EKF bias starts at 0 rad/s");
    }

    landmark_correction_enable_topic_ = declare_parameter(
        "landmark_correction_enable_topic",
        std::string("task_manager_node/using_vmarker_localization"));
    landmark_observation_topic_ = declare_parameter(
        "landmark_observation_topic",
        std::string("~/landmark_observation"));
    pose_command_topic_ = declare_parameter(
        "pose_command_topic",
        std::string("~/set_pose"));

    logger_->info("Pose command topic: '{}'", pose_command_topic_);
    logger_->info("Landmark correction topics: enable='{}', observation='{}'",
                  landmark_correction_enable_topic_, landmark_observation_topic_);
}

void OdomEstimation::setupPublishers()
{
    // Create pose publisher
    pose_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(10));
    
    // Create marker publisher for OBJ visualization
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/robot_marker", rclcpp::QoS(10));
    
    landmark_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/landmark_pose", rclcpp::QoS(10));
    gyro_bias_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "~/gyro_bias", rclcpp::QoS(10));

    logger_->info("Publishers created");
}

void OdomEstimation::setupSubscribers()
{
    landmark_observation_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        landmark_observation_topic_, rclcpp::QoS(10),
        std::bind(&OdomEstimation::landmarkObservationCallback, this, std::placeholders::_1));

    // Setup odom and IMU subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/motor_driver_node/odom", 10,
        std::bind(&OdomEstimation::wheelOdomCallback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        // "/imu_driver_node/imu/corrected_data", 10,
        std::bind(&OdomEstimation::imuOdomCallback, this, std::placeholders::_1));

    pose_command_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_command_topic_, rclcpp::QoS(10),
        std::bind(&OdomEstimation::poseCommandCallback, this, std::placeholders::_1));

    landmark_correction_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        landmark_correction_enable_topic_, rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&OdomEstimation::landmarkCorrectionEnableCallback, this, std::placeholders::_1));
    logger_->info("Created landmark correction enable subscriber");

    logger_->info("Subscribers created");
}

bool OdomEstimation::initialize()
{
    logger_->info("Starting localizer initialization process...");

    // Create appropriate localizer based on odom_estimator_type parameter
    std::string odom_estimator_type_str = odomEstimatorTypeToString(odom_estimator_type_);
    logger_->info("Requested localizer type: '{}'",  odom_estimator_type_str);

    // Create diagnostic manager with lifecycle node
    double robot_pose_timeout_ms = declare_parameter("robot_pose_timeout", 100.0);
    logger_->info("Robot pose timeout set to {:.3f} ms", robot_pose_timeout_ms);
    
    diagnostic_manager_ = std::make_shared<diagnostic::DiagnosticManager>(
        this,  // lifecycle node pointer
        this->get_name(),  // module name
        "diagnostic_msg",  // topic name
        20.0  // 20Hz publish rate
    );

    // Register pose timeout checker
    std::shared_ptr<std::optional<rclcpp::Time>> last_pose_time = 
        std::make_shared<std::optional<rclcpp::Time>>();
    
    diagnostic_manager_->registerDiagnosticChecker(
        [this, last_pose_time, robot_pose_timeout_ms]() -> std::vector<gole_interfaces::msg::Diagnostic> {
            std::vector<gole_interfaces::msg::Diagnostic> diagnostics;
            
            if (!last_pose_time->has_value()) {
                // Pose never published
                gole_interfaces::msg::Diagnostic diag;
                diag.level = static_cast<int8_t>(diagnostic::DiagnosticLevel::ERROR);
                diag.diagnostic_id = static_cast<int32_t>(diagnostic::DiagnosticID::ROBOT_POSE_TIMEOUT);
                diag.diagnostic_name = "ROBOT_POSE_TIMEOUT";
                diagnostics.push_back(diag);
                return diagnostics;
            }
            
            auto time_since_last = (get_clock()->now() - **last_pose_time).seconds() * 1000.0;
            if (time_since_last > robot_pose_timeout_ms) {
                // Pose timeout detected
                gole_interfaces::msg::Diagnostic diag;
                diag.level = static_cast<int8_t>(diagnostic::DiagnosticLevel::ERROR);
                diag.diagnostic_id = static_cast<int32_t>(diagnostic::DiagnosticID::ROBOT_POSE_TIMEOUT);
                diag.diagnostic_name = "ROBOT_POSE_TIMEOUT";
                diagnostics.push_back(diag);
                return diagnostics;
            }
            
            // No timeout, clear any existing error
            diagnostic_manager_->clear(diagnostic::DiagnosticID::ROBOT_POSE_TIMEOUT);
            return diagnostics;  // Empty vector
        }
    );

    odom_estimator_ = OdomEstimatorFactory::createOdomEstimator(
        odom_estimator_type_str, this, logger_, *params_);

    logger_->info("Initializing localizer components...");
    odom_estimator_->initialize();

    logger_->info("Setting default initial pose...");
    defaultInitPose();

    logger_->info("Registering notification handlers...");
    
    // Register notification callback with pose time tracking
    odom_estimator_->setNotifyCallback(
        [this, last_pose_time](const std::string& event, const std::any& data) {
            // Update last pose time
            *last_pose_time = this->get_clock()->now();
            // Handle the notification
            this->handleNotification(event, data);
        }
    );

    logger_->info("Localizer successfully started in '{}' mode", odom_estimator_type_str);
    logger_->info("OdomEstimation initialization completed successfully");

    return true;
}

void OdomEstimation::defaultInitPose()
{
    logger_->info("Setting default initial pose (0, 0, 0)");
    
    // Create default initial pose message
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    
    // Set header
    msg->header.stamp = this->now();
    msg->header.frame_id = "odom";
    
    // Set pose - position at origin (0, 0, 0)
    msg->pose.pose.position.x = 0.0;
    msg->pose.pose.position.y = 0.0;
    msg->pose.pose.position.z = 0.0;
    
    // Set orientation - no rotation (identity quaternion)
    msg->pose.pose.orientation.x = 0.0;
    msg->pose.pose.orientation.y = 0.0;
    msg->pose.pose.orientation.z = 0.0;
    msg->pose.pose.orientation.w = 1.0;
    
    // Set covariance matrix - small values indicating high confidence in default pose
    for (int i = 0; i < 36; i++) {
        msg->pose.covariance[i] = 0.0;
    }
    // Set diagonal elements for position (x, y) and orientation (yaw)
    msg->pose.covariance[0] = 0.1;   // x variance
    msg->pose.covariance[7] = 0.1;   // y variance 
    msg->pose.covariance[14] = params_->enable_3d_estimation
        ? params_->initial_z_covariance : 0.0;
    msg->pose.covariance[21] = params_->enable_3d_estimation
        ? params_->initial_roll_pitch_covariance : 0.0;
    msg->pose.covariance[28] = params_->enable_3d_estimation
        ? params_->initial_roll_pitch_covariance : 0.0;
    msg->pose.covariance[35] = params_->initial_yaw_covariance;
    
    // Process the default initial pose through localizer
    odom_estimator_->processInitialPose(msg);
    
    logger_->info("Default initial pose set successfully");
}

// -----------------------------------------------------------------------------
// Callback Functions
// -----------------------------------------------------------------------------

void OdomEstimation::timerCallback()
{
    auto now = std::chrono::steady_clock::now();
    // Only publish odom->base_link after the estimator is initialized.
    if (!odom_estimator_) {
        if ((now - transform_last_log_time_) > std::chrono::seconds(1)) {
            logger_->warn("[timerCallback] Localizer instance is null, cannot publish transform");
            transform_last_log_time_ = now;
        }

        return;
    }

    if (!odom_estimator_->isInitialized()) {
        if ((now - transform_last_log_time_) > std::chrono::seconds(1)) {
            logger_->warn(
                "[timerCallback] Localizer not initialized yet, skipping transform publish");
            transform_last_log_time_ = now;
        }

        return;
    }

    if (!tf_broadcaster_) {
        if ((now - transform_last_log_time_) > std::chrono::seconds(1)) {
            logger_->error("[timerCallback] TF Broadcaster not initialized, cannot publish transform");
            transform_last_log_time_ = now;
        }

        return;
    }
    
    // State time is owned by sensor-time propagation. A wall timer must never
    // overwrite it, otherwise delayed/bagged sensor measurements become stale.

    // Only odom->base_link is published here. Global localization owns
    // map->odom when that transform is needed.
    auto odom_to_base_link = odom_estimator_->getOdomToBaselinkTransform();
    odom_to_base_link.header.stamp = this->now();
    tf_broadcaster_->sendTransform(odom_to_base_link);

    logger_->trace("Published odom->base_link transform via timer");
    
    if (using_landmark_correction_ && landmark_pose_pub_ && landmark_pose_pub_->is_activated() &&
        odom_estimator_->isLandmarkInitialized())
    {
        auto landmark_state = odom_estimator_->getLandmarkState();
        if (landmark_state.has_value()) {
            landmark_pose_pub_->publish(landmark_state.value());
        }
    }
}

void OdomEstimation::landmarkObservationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!odom_estimator_) {
        return;
    }

    logger_->trace("Received landmark observation");
    if (!using_landmark_correction_) {
        return;
    }

    odom_estimator_->processLandmarkObservation(msg);
}

void OdomEstimation::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    logger_->debug("Odom callback received odom data with timestamp: {:.9f}", 
                  msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    odom_estimator_->processWheel(msg);
}

void OdomEstimation::imuOdomCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (!odom_estimator_ || !tf_buffer_ || !msg) {
        return;
    }
    logger_->debug("IMU callback received IMU data with timestamp: {:.9f}", 
                  msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);

    if (msg->header.frame_id.empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_imu_tf_warning_time_ > std::chrono::seconds(1)) {
            logger_->warn("Rejecting /imu/data: header.frame_id is empty");
            last_imu_tf_warning_time_ = now;
        }
        return;
    }

    auto transformed = std::make_shared<sensor_msgs::msg::Imu>(*msg);
    try {
        if (msg->header.frame_id != params_->imu_target_frame) {
            const auto transform = tf_buffer_->lookupTransform(
                params_->imu_target_frame,
                msg->header.frame_id,
                rclcpp::Time(msg->header.stamp),
                rclcpp::Duration::from_seconds(params_->imu_tf_lookup_timeout));
            transformed->angular_velocity = rotateAngularVelocity(
                msg->angular_velocity, transform.transform.rotation);
            transformed->angular_velocity_covariance =
                rotateAngularVelocityCovariance(
                    msg->angular_velocity_covariance,
                    transform.transform.rotation);
        }
        transformed->header.frame_id = params_->imu_target_frame;
        odom_estimator_->processImu(transformed);
    } catch (const std::exception& error) {
        const auto now = std::chrono::steady_clock::now();
        if (now - last_imu_tf_warning_time_ > std::chrono::seconds(1)) {
            logger_->warn(
                "Rejecting /imu/data: cannot rotate angular velocity from '{}' "
                "to '{}': {}. Fix the URDF/static TF for the physical IMU mount.",
                msg->header.frame_id,
                params_->imu_target_frame,
                error.what());
            last_imu_tf_warning_time_ = now;
        }
    }
}

void OdomEstimation::poseCommandCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    if (!odom_estimator_) {
        logger_->warn("OdomEstimator instance is null, cannot publish transform");
        return;
    }

    logger_->info("Received localization pose command");
    odom_estimator_->processInitialPose(msg);
}

void OdomEstimation::landmarkCorrectionEnableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool new_value = msg->data;
    bool old_value = using_landmark_correction_;
    
    using_landmark_correction_ = new_value;
    logger_->info("landmark correction updated to: {}", using_landmark_correction_ ? "true" : "false");
    
    if (odom_estimator_ && old_value != new_value) {
        if (new_value) {
            odom_estimator_->enableLandmarkCorrection();
            logger_->info("Enabled landmark correction");
        } else {
            odom_estimator_->disableLandmarkCorrection();
            logger_->info("Disabled landmark correction");
        }
    }
}


// =============================================================================
// Notification System Implementation
// =============================================================================
void OdomEstimation::publishObjMarker(const nav_msgs::msg::Odometry& odom)
{
    if (!marker_pub_ || !marker_pub_->is_activated()) {
        return;
    }

    auto marker = visualization_msgs::msg::Marker();
    
    // Basic marker properties
    marker.header.frame_id = odom.header.frame_id;
    marker.header.stamp = this->now();
    marker.ns = "robot_mesh";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Set pose from robot pose
    marker.pose = odom.pose.pose;
    
    // Set scale (adjust as needed)
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    
    // Set color (will be overridden by MTL file if available)
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0f;
    
    // Set mesh resource - point to the OBJ file in the models directory
    try {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("odom_estimation");
        marker.mesh_resource = "file://" + package_share_directory + "/models/gl_shell.obj";
        
        // MTL file should be automatically loaded if it exists alongside the OBJ file
        marker.mesh_use_embedded_materials = true;
        
        // Publish the marker
        marker_pub_->publish(marker);
        
        logger_->debug("[publishObjMarker] Published OBJ marker at position: x={:.3f}, y={:.3f}, z={:.3f}", 
                      marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
    } catch (const std::exception& e) {
        logger_->error("Failed to publish OBJ marker: {}", e.what());
    }
}

void OdomEstimation::handleNotification(const std::string& event_name, const std::any& data)
{
    logger_->debug("[handleNotification] Received localizer notification: {}", event_name);
    nav_msgs::msg::Odometry odom;
    try {
        if (event_name == ODOMETRY_UPDATED_EVENT
            || event_name == INIT_POSE_PROCESSED_EVENT
            || event_name == LANDMARK_PROCESSED_EVENT)
        {
            odom = std::any_cast<nav_msgs::msg::Odometry>(data);
        } 
        // else if (event_name == LASER_PROCESSED_EVENT) {
        //     current_pose = std::any_cast<geometry_msgs::msg::PoseWithCovarianceStamped>(data);
        // }
        else {
            logger_->warn("Unhandled event name in notification: {}", event_name);
            return;
        }
        
        auto state = odom_estimator_->getCurrentState();
        logger_->debug("[handleNotification] updated robot pose: x={:.3f}, y={:.3f}, yaw={:.3f}°", 
                      state.mu(0), state.mu(1), state.mu(2) * 180.0 / M_PI);

        if (gyro_bias_pub_ && gyro_bias_pub_->is_activated()) {
            geometry_msgs::msg::Vector3Stamped gyro_bias;
            gyro_bias.header = odom.header;
            gyro_bias.vector.x = state.mu(9);
            gyro_bias.vector.y = state.mu(10);
            gyro_bias.vector.z = state.mu(5);
            gyro_bias_pub_->publish(gyro_bias);
        }

    } catch (const std::exception& e) {
        logger_->error("Error handling localizer notification '{}': {}", event_name, e.what());
    }

    // Publish the updated pose
    if (pose_pub_ && pose_pub_->is_activated()) {
        pose_pub_->publish(odom);
        
        // Publish OBJ marker at robot pose
        publishObjMarker(odom);
    }
}


} // namespace odom_estimation
} // namespace gole
