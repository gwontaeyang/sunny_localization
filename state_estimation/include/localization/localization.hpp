#ifndef LOCALIZATION_LOCALIZATION_HPP_
#define LOCALIZATION_LOCALIZATION_HPP_

#include <memory>
#include <string>
#include <functional>
#include <unordered_map>
#include <any>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"
#include "message_filters/simple_filter.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "logger/logger.hpp"
#include "diagnostic/diagnostic_manager.hpp"

#include "localization/localizer/localizer_base.hpp"

namespace gole {
namespace localization {

// Wrapper class for SimpleFilter to expose signalMessage as public
template<typename M>
class SimpleFilterWrapper : public message_filters::SimpleFilter<M>
{
public:
    void add(const typename message_filters::SimpleFilter<M>::MConstPtr& msg)
    {
        this->signalMessage(msg);
    }
};

/**
 * @brief Abstract base class for all localization implementations
 * 
 * This class provides a common interface for different localization algorithms.
 * All concrete localization implementations should inherit from this class.
 */
class Localization : public rclcpp_lifecycle::LifecycleNode {
public:
    /**
     * @brief Constructor
     */
    Localization();

    /**
     * @brief Virtual destructor
     */
    virtual ~Localization() = default;

protected:
    // Lifecycle node callbacks
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state) override;


    bool initialize();
    // bool start();
    // bool stop();
    // bool reset();

    geometry_msgs::msg::PoseWithCovarianceStamped getCurrentPose() const;

private:
    // // Core configuration
    std::unique_ptr<LocalizerBase> localizer_;
    std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager_;
    std::shared_ptr<spdlog::logger> logger_;
    LocalizerType localizer_type_;
    
    // Parameters
    std::shared_ptr<Params> params_;

    // Publishers
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr landmark_pose_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr landmark_observation_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_forward_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr landmark_correction_enable_sub_;

    // Odom and IMU subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Transform publishing timer
    rclcpp::TimerBase::SharedPtr transform_timer_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::chrono::steady_clock::time_point transform_last_log_time_;

    // Configuration parameters
    bool is_forward_;  // Forward/backward direction flag
    bool using_landmark_correction_;
    std::string pose_command_topic_;
    std::string landmark_observation_topic_;
    std::string landmark_correction_enable_topic_;
    
    // Stationary detection
    std::optional<std::pair<double, double>> previous_pose_;  // Previous (x, y) position for stationary detection
    
    // Setup methods
    void setupParameters();
    void setupPublishers();
    void setupSubscribers();
    void defaultInitPose();

    // Callback functions
    void timerCallback();
    void landmarkObservationCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuOdomCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void poseCommandCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void isForwardCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void landmarkCorrectionEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);

    // Marker publishing
    void publishObjMarker(const nav_msgs::msg::Odometry& odom);
    
    // Direct handler for localizer notifications
    void handleNotification(const std::string& event_name, const std::any& data);
};

} // namespace localization
} // namespace gole

#endif  // LOCALIZATION_LOCALIZATION_HPP_
