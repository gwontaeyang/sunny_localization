#ifndef ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_EKF_HPP_
#define ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_EKF_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <optional>
#include <vector>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "odom_estimation/odom_estimator/odom_estimator_base.hpp"

namespace gole {
namespace odom_estimation {

/**
 * @brief Simple example odom_estimator implementation for demonstration
 * 
 * This is a basic implementation that shows how to extend OdomEstimatorBase.
 * In a real scenario, this would contain actual localization algorithms.
 */
class OdomEstimatorEKF : public OdomEstimatorBase {
public:
    OdomEstimatorEKF(rclcpp_lifecycle::LifecycleNode* node,
                 std::shared_ptr<spdlog::logger> logger,
                 std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager,
                 const Params& params);
    ~OdomEstimatorEKF() override = default;

    // bool start() override;
    // bool stop() override;
    // bool reset() override;

    void processInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) override;
    void processLandmarkObservation(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr observation) override;
    void processWheel(const nav_msgs::msg::Odometry::SharedPtr odom) override;
    void processImu(const sensor_msgs::msg::Imu::SharedPtr imu) override;
    
    void enableLandmarkCorrection() override;
    void disableLandmarkCorrection() override;
    
    /**
     * @brief Check if external landmark is initialized in the EKF state.
     * @return True if landmark correction is active and the landmark is initialized.
     */
    bool isLandmarkInitialized() const override;
    
private:    
    void findInitPose(const geometry_msgs::msg::PoseWithCovarianceStamped& initial_guess);
    void processSensorFusionTick();
    
    bool prediction(const geometry_msgs::msg::Twist::SharedPtr cmd_vel, double dt) override;
    /**
     * @brief Correction step of EKF using external landmark measurement.
     * @param observation Landmark pose measurement in base_link frame.
     * @return true if correction was successful
     */
    bool correctionLandmarkObservation(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr observation);
    /**
     * @brief Correction step of EKF using IMU measurement (yaw only)
     * @param imu IMU measurement
     * @return true if correction was successful
     */
    bool correctionImu(const sensor_msgs::msg::Imu::SharedPtr imu);
    /**
     * @brief Correction step of EKF using odometry measurement (x, y only)
     * @param odom Odometry measurement
     * @return true if correction was successful
     */
    bool correctionWheel(const nav_msgs::msg::Odometry::SharedPtr odom);


//*********************
// util functions
//*********************
private:
    rclcpp::TimerBase::SharedPtr sensor_fusion_timer_;
    std::mutex measurement_mutex_;
    nav_msgs::msg::Odometry::SharedPtr latest_wheel_odom_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> wheel_odom_queue_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    bool has_new_wheel_{false};
    bool has_new_imu_{false};

    // Landmark covariance convergence tracking
    std::vector<double> lm_covariance_history_;  // Store recent covariance norms
    bool landmark_initialized_{false};
    static constexpr int CONVERGENCE_WINDOW_SIZE = 10;  // Number of observations to track
    static constexpr double CONVERGENCE_THRESHOLD = 0.40;  // Max change ratio for stability
};

} // namespace odom_estimation
} // namespace gole

#endif // ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_EKF_HPP_
