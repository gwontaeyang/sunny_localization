#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rmw/qos_profiles.h"

#include "odom_estimation/odom_estimation.hpp"
#include "odom_estimation/odom_estimator_factory.hpp"

namespace gole {
namespace odom_estimation {

OdomEstimation::OdomEstimation()
    : rclcpp_lifecycle::LifecycleNode("odom_estimation_node")
    , is_forward_(true)
    , using_landmark_correction_(false)
{
    logger_ = gole::logger::Logger::createPackageLogger(this->get_name(), spdlog::level::level_enum::info);
    logger_->info("odom estimation starting...");
    transform_last_log_time_ = std::chrono::steady_clock::now();
}

// -----------------------------------------------------------------------------
// Lifecycle Functions
// -----------------------------------------------------------------------------

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OdomEstimation::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    try {
        logger_->info("Configuring Odom Estimation...");
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

        logger_->info("Odom Estimation configured successfully");
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
    logger_->info("Activating Odom Estimation...");

    // Activate publishers
    pose_pub_->on_activate();
    marker_pub_->on_activate();
    landmark_pose_pub_->on_activate();
    
    // Activate diagnostic manager
    if (diagnostic_manager_) {
        diagnostic_manager_->activate();
    }

    // Create and start timer only when activated
    transform_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&OdomEstimation::timerCallback, this)
    );

    logger_->info("Odom Estimation activated successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OdomEstimation::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    logger_->info("Deactivating Odom Estimation...");

    // Stop timers when deactivating
    if (transform_timer_) {
        transform_timer_->cancel();
        transform_timer_.reset();
    }

    // Deactivate publishers
    pose_pub_->on_deactivate();
    marker_pub_->on_deactivate();
    landmark_pose_pub_->on_deactivate();
    
    // Deactivate diagnostic manager
    if (diagnostic_manager_) {
        diagnostic_manager_->deactivate();
    }

    logger_->info("Odom Estimation deactivated successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OdomEstimation::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    logger_->info("Cleaning up Odom Estimation...");
    
    // Reset all components
    odom_sub_.reset();
    imu_sub_.reset();
    pose_command_sub_.reset();
    odom_estimator_.reset();

    logger_->info("Odom Estimation cleaned up successfully");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
OdomEstimation::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
    logger_->info("Shutting down Odom Estimation...");
    
    // Reset all components
    odom_sub_.reset();
    imu_sub_.reset();
    pose_command_sub_.reset();
    odom_estimator_.reset();

    logger_->info("Odom Estimation shut down successfully");
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
    logger_->info("Odom Estimation parameter 'odom_estimator_type' set to '{}'", odom_estimator_type_str);
    
    // General localization parameters
    params_->is_forward = true;  // Will be updated by topic callback
    params_->using_landmark_correction = false;  // Will be updated by topic callback

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

    // Create is_forward subscriber
    is_forward_sub_ = create_subscription<std_msgs::msg::Bool>(
        "task_manager_node/is_forward", rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&OdomEstimation::isForwardCallback, this, std::placeholders::_1));
    logger_->info("Created is_forward subscriber");

    landmark_correction_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        landmark_correction_enable_topic_, rclcpp::QoS(10).reliable().transient_local(),
        std::bind(&OdomEstimation::landmarkCorrectionEnableCallback, this, std::placeholders::_1));
    logger_->info("Created landmark correction enable subscriber");

    logger_->info("Subscribers created with synchronized odom and IMU filter");
}

bool OdomEstimation::initialize()
{
    logger_->info("Starting odom_estimator initialization process...");

    // Create appropriate odom_estimator based on odom_estimator_type parameter
    std::string odom_estimator_type_str = odomEstimatorTypeToString(odom_estimator_type_);
    logger_->info("Requested odom_estimator type: '{}'",  odom_estimator_type_str);

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

    odom_estimator_ = OdomEstimatorFactory::createOdomEstimator(odom_estimator_type_str, this, logger_, diagnostic_manager_, *params_);

    logger_->info("Initializing odom_estimator components...");
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

    logger_->info("OdomEstimator successfully started in '{}' mode", odom_estimator_type_str);
    logger_->info("Localization initialization completed successfully");

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
    msg->pose.covariance[35] = 0.1;  // yaw variance
    
    // Process the default initial pose through odom_estimator
    odom_estimator_->processInitialPose(msg);
    
    logger_->info("Default initial pose set successfully");
}

// -----------------------------------------------------------------------------
// Callback Functions
// -----------------------------------------------------------------------------

void OdomEstimation::timerCallback()
{
    auto now = std::chrono::steady_clock::now();
    // Only publish map->odom transform if odom_estimator is initialized
    if (!odom_estimator_) {
        if ((now - transform_last_log_time_) > std::chrono::seconds(1)) {
            logger_->warn("[timerCallback] OdomEstimator instance is null, cannot publish transform");
            transform_last_log_time_ = now;
        }

        return;
    }

    if(!odom_estimator_->isInitialized() || !odom_estimator_->isMapToOdomInitialized()) {
        if ((now - transform_last_log_time_) > std::chrono::seconds(1)) {
            logger_->warn("[timerCallback] OdomEstimator not initialized yet or Map to Odom transform not initialized yet, skipping transform publish");
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
    
    auto current_ros_time = this->now();
    auto current_state = odom_estimator_->getCurrentState();
    
    // If state time is not initialized (0), reset it to current time
    if (current_state.time.nanoseconds() == 0) {
        odom_estimator_->updateStateTime(current_ros_time);
        logger_->debug("[timerCallback] Initialized state time to current ROS time: {:.9f}s", current_ros_time.seconds());
        return;
    }
    
    auto time_since_last_update = (current_ros_time - current_state.time).seconds() * 1000.0; // Convert to ms
    
    if (time_since_last_update > 50.0) {
        odom_estimator_->updateStateTime(current_ros_time);
        logger_->debug("[timerCallback] Updated stale state time: {:.9f}ms since last update, state_time={:.9f}s, current_time={:.9f}s", 
                      time_since_last_update, current_state.time.seconds(), current_ros_time.seconds());
    }
    
    // Stationary detection: Check if position change is less than 20mm (0.02m)
    double current_x = current_state.mu(0);
    double current_y = current_state.mu(1);
    
    if (previous_pose_.has_value()) {
        double dx = current_x - previous_pose_->first;
        double dy = current_y - previous_pose_->second;
        double distance = std::hypot(dx, dy);
        
        if (distance > 0.005) {  // 5mm threshold
            logger_->debug("[timerCallback] Robot is stationary: position change = {:.9f}m (> 5mm)", distance);
        }
    }
    
    // Update previous pose for next iteration
    previous_pose_ = std::make_pair(current_x, current_y);

    // Publish transforms: map->odom and odom->base_link
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    
    // Get map->odom transform
    // auto map_to_odom = odom_estimator_->getMapToOdomTransform();
    // map_to_odom.header.stamp = this->now();
    // transforms.push_back(map_to_odom);
    
    // Get odom->base_link transform
    auto odom_to_base_link = odom_estimator_->getOdomToBaselinkTransform();
    odom_to_base_link.header.stamp = this->now();
    transforms.push_back(odom_to_base_link);
    
    // Send all transforms at once
    tf_broadcaster_->sendTransform(transforms);

    logger_->trace("Published map->odom and odom->base_link transforms via timer");
    
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
    logger_->debug("IMU callback received IMU data with timestamp: {:.9f}", 
                  msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    odom_estimator_->processImu(msg);
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

void OdomEstimation::isForwardCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    is_forward_ = msg->data;
    logger_->info("is_forward updated to: {}", is_forward_ ? "true" : "false");
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
    logger_->debug("[handleNotification] Received odom_estimator notification: {}", event_name);
    nav_msgs::msg::Odometry odom;
    try {
        if (event_name == WHEEL_PROCESSED_EVENT
            || event_name == INIT_POSE_PROCESSED_EVENT
            || event_name == IMU_PROCESSED_EVENT
            || event_name == POINT_CLOUD_PROCESSED_EVENT)
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

    } catch (const std::exception& e) {
        logger_->error("Error handling odom_estimator notification '{}': {}", event_name, e.what());
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
