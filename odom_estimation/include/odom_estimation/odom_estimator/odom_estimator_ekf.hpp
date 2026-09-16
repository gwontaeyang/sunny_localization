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

/** @brief Wheel/IMU EKF odometry estimator with optional landmark correction. */
class OdomEstimatorEKF : public OdomEstimatorBase {
public:
    OdomEstimatorEKF(rclcpp_lifecycle::LifecycleNode* node,
                 std::shared_ptr<spdlog::logger> logger,
                 const Params& params);
    ~OdomEstimatorEKF() override = default;

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
     * @brief Correct the gyro-z rate state and stationary 3-axis gyro biases
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
    std::deque<nav_msgs::msg::Odometry::SharedPtr> wheel_odom_queue_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    bool first_sensor_time_aligned_{false};
    bool has_wheel_measurement_{false};
    bool gyro_bias_initialized_{false};
    std::optional<rclcpp::Time> last_received_wheel_stamp_;
    std::optional<rclcpp::Time> last_received_imu_stamp_;
    std::optional<double> last_accepted_wheel_v_;
    std::optional<double> latest_wheel_angular_z_;
    std::optional<double> last_accepted_gyro_z_;
    geometry_msgs::msg::Vector3 last_body_angular_velocity_{};
    std::optional<rclcpp::Time> stationary_start_time_;
    Eigen::Vector3d stationary_gyro_sum_{Eigen::Vector3d::Zero()};
    std::size_t stationary_gyro_samples_{0};

    // Landmark covariance convergence tracking
    std::vector<double> lm_covariance_history_;  // Store recent covariance norms
    bool landmark_initialized_{false};
    static constexpr int CONVERGENCE_WINDOW_SIZE = 10;  // Number of observations to track
    static constexpr double CONVERGENCE_THRESHOLD = 0.40;  // Max change ratio for stability
};

} // namespace odom_estimation
} // namespace gole

#endif // ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_EKF_HPP_
