#ifndef LOCALIZATION_LOCALIZER_LOCALIZER_BASE_HPP_
#define LOCALIZATION_LOCALIZER_LOCALIZER_BASE_HPP_

#include <any>
#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <functional>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "logger/logger.hpp"
#include "diagnostic/diagnostic_manager.hpp"

#include "localization/localizer/localizer_types.hpp"

namespace gole {
namespace localization {

/**
 * @brief Abstract base class for all localizer implementations
 * 
 * This class provides a common interface for different localization algorithms.
 * All concrete localizer implementations should inherit from this class.
 */
class LocalizerBase {
public:
    /**
     * @brief Constructor
     */
    LocalizerBase(rclcpp_lifecycle::LifecycleNode* node,
                  std::shared_ptr<spdlog::logger> logger,
                  std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager,
                  const Params& params);

    /**
     * @brief Virtual destructor
     */
    virtual ~LocalizerBase() = default;

public:
    /**
     * @brief Initialize the localizer - to be implemented by derived classes
     * @return True if initialization is successful
     */
    bool initialize();

    /**
     * @brief Set initial pose - to be implemented by derived classes
     * @param pose Initial pose with covariance
     */
    virtual void processInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) = 0;

    /**
     * @brief Process an external landmark observation in the base_link frame.
     * @param observation Landmark pose measurement with covariance.
     */
    virtual void processLandmarkObservation(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr observation) = 0;

    /**
     * @brief Process odometry data - to be implemented by derived classes
     * @param odom Odometry message
     */
    virtual void processWheel(const nav_msgs::msg::Odometry::SharedPtr odom) = 0;

    /**
     * @brief Process IMU data - to be implemented by derived classes
     * @param imu IMU message
     */
    virtual void processImu(const sensor_msgs::msg::Imu::SharedPtr imu) = 0;

    /**
     * @brief Check if the localizer is initialized
     * @return True if initialized
     */
    inline bool isInitialized() const { return is_init_.load(); }
    inline bool isMapToOdomInitialized() const { return map_to_odom_initialized_; }
    // inline bool isDoingStateEstimation() const { return is_doing_state_estimation_.load(); }
    
    /**
     * @brief Check if the external landmark is initialized in the state.
     * @return True if landmark correction is active and the landmark is initialized.
     */
    virtual bool isLandmarkInitialized() const = 0;
    
    /**
     * @brief Get map to odom transform safely (thread-safe copy)
     * @return Thread-safe copy of the transform
     */
    geometry_msgs::msg::TransformStamped getMapToOdomTransform() {
        std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
        return map_to_odom_transform_;
    }

    /**
     * @brief Get odom to base_link transform
     * @return Transform from odom to base_link
     */
    geometry_msgs::msg::TransformStamped getOdomToBaselinkTransform();

    /**
     * @brief Get current state estimate
     * @return Current state
     */
    State getCurrentState();
    
    /**
     * @brief Update state timestamp without modifying state estimate
     * @param new_time New timestamp for the state
     */
    void updateStateTime(const rclcpp::Time& new_time);
    
    /**
     * @brief Get external landmark state (position, orientation, covariance).
     * @return Optional containing PoseWithCovarianceStamped if correction mode is active and landmark exists.
     */
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> getLandmarkState();

    /**
     * @brief Enable external landmark correction.
     */
    virtual void enableLandmarkCorrection() = 0;
    
    /**
     * @brief Disable external landmark correction.
     */
    virtual void disableLandmarkCorrection() = 0;
    
    /**
     * @brief Check if external landmark correction is active.
     * @return true if active
     */
    bool isLandmarkCorrectionEnabled() const { return is_landmark_correction_enabled_.load(); }


    // Notification system
    using NotifyCallback = std::function<void(const std::string&, const std::any&)>;
    
    /**
     * @brief Set notification callback for processing complete events
     * @param callback Function to call when processing is complete
     */
    void setNotifyCallback(const NotifyCallback& callback) { notify_callback_ = callback; }

protected:
    rclcpp_lifecycle::LifecycleNode* node_;
    std::shared_ptr<spdlog::logger> logger_;
    std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager_;
    
    // Parameters
    const Params& params_;
    
    // Frame IDs
    std::string map_frame_id_;
    std::string odom_frame_id_;
    std::string base_link_frame_id_;

    // State management
    std::atomic<bool> is_init_;
    std::atomic<bool> is_doing_state_estimation_;
    std::atomic<bool> is_landmark_correction_enabled_;
    
    // Map to Odom transform (for localization correction)
    geometry_msgs::msg::TransformStamped map_to_odom_transform_;
    bool map_to_odom_initialized_;
    
    // Current state
    State current_state_;
    Eigen::MatrixXd Qu_;

    // Update state
    Eigen::MatrixXd Qr_;

    // mutex
    std::mutex state_mutex_;
    std::mutex map_to_odom_mutex_;

    // Notification callback
    NotifyCallback notify_callback_;
    
    virtual bool prediction(const geometry_msgs::msg::Twist::SharedPtr cmd_vel, double dt) = 0;
    // correctionOdom(), correctionImu(), correctionWheel(), correctionLandmarkObservation()

    /**
     * @brief Trigger notification to parent class
     * @param event_name Name of the event
     * @param data Event data
     */
    void triggerNotify(const std::string& event_name, const std::any& data) {
        if (notify_callback_) {
            notify_callback_(event_name, data);
        }
    }

    nav_msgs::msg::Odometry state2Msg();
    void updateMapToOdomTransform(Eigen::Isometry3d& map_to_odom, rclcpp::Time time);
    void updateMapToOdomTransform(const nav_msgs::msg::Odometry::SharedPtr odom);
    void updateMapToOdomTransform(Eigen::Vector3d pos, Eigen::Quaterniond angle, rclcpp::Time time);
};

} // namespace localization
} // namespace gole

#endif // LOCALIZATION_LOCALIZER_LOCALIZER_BASE_HPP_
