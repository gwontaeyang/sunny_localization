#ifndef ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_TYPES_HPP_
#define ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_TYPES_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <stdexcept>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/time.hpp"

namespace gole {
namespace odom_estimation {

/**
 * @brief Localization system parameters
 */
struct Params {
    // Optional offline gyro calibration used to seed the EKF bias state.
    bool load_initial_gyro_bias = false;
    std::string gyro_bias_yaml_path = "~/.ros/imu_calibration.yaml";
    bool allow_initial_gyro_bias_fallback = false;
    std::string gyro_bias_fallback_yaml_path;
    double max_initial_gyro_bias_abs = 0.05;
    double initial_gyro_bias_x = 0.0;
    double initial_gyro_bias_y = 0.0;
    double initial_gyro_bias_z = 0.0;

    // Sensor-frame handling
    std::string imu_target_frame = "base_link";
    double imu_tf_lookup_timeout = 0.05;

    // Wheel forward-velocity measurement
    bool use_wheel_message_covariance = true;
    double wheel_velocity_noise = 0.0016;
    double wheel_min_velocity_variance = 1.0e-6;

    // Base-frame 3-axis gyro attitude propagation and z-rate correction.
    // sensor_msgs/Imu.orientation is deliberately not fused.
    bool enable_3d_estimation = false;
    bool use_imu_message_covariance = true;
    double gyro_noise = 7.615435494667714e-5;
    double gyro_min_variance = 1.0e-8;
    double gyro_bias_random_walk = 7.615435494667715e-9;
    double initial_yaw_covariance = 0.030461741978670857;
    double initial_gyro_bias_covariance = 7.615435494667714e-5;
    double initial_z_covariance = 0.04;  // 0.2 m std
    double initial_roll_pitch_covariance = 0.007615435494667714;  // 5 deg std
    double vertical_motion_noise = 0.0025;  // (m/s)^2
    double roll_pitch_process_noise = 7.615435494667714e-5;  // rad^2/s
    double max_prediction_dt = 0.5;
    bool reject_zero_timestamp = true;

    // Stationary gyro-bias calibration/update
    bool gyro_initial_bias_enabled = true;
    double gyro_initial_bias_duration = 2.0;
    int gyro_initial_bias_min_samples = 100;
    double gyro_stationary_threshold = 0.03490658503988659;
    double wheel_stationary_threshold = 0.005;
    double wheel_stationary_angular_threshold = 0.001;
    double stationary_gyro_bias_variance = 1.2184696791468346e-5;

    // The planar model has zero mean lateral velocity, with this non-zero
    // process variance retaining uncertainty for real lateral slip.
    bool vy_zero_constraint_enabled = true;
    double vy_zero_constraint_noise = 0.04;
};

/**
 * @brief Enumeration of supported odom_estimator types
 */
enum class OdomEstimatorType {
    EKF
};

/**
 * @brief Convert odom_estimator type to string
 * @param type OdomEstimator type
 * @return String representation of the odom_estimator type
 */
inline std::string odomEstimatorTypeToString(OdomEstimatorType type) {
    switch (type) {
        case OdomEstimatorType::EKF:
            return "EKF";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief Convert string to odom_estimator type
 * @param type_str String representation of odom_estimator type
 * @return OdomEstimator type
 */
inline OdomEstimatorType stringToOdomEstimatorType(const std::string& type_str) {
    if (type_str == "EKF") {
        return OdomEstimatorType::EKF;
    } else {
        throw std::invalid_argument(
            "Unknown odom_estimator type: " + type_str +
            ". The only implemented option is 'EKF'");
    }
}

struct State
{
    rclcpp::Time time;  // Use ROS2 time directly for better precision
    Eigen::VectorXd mu;
    Eigen::MatrixXd sigma;
    
    // Default constructor
    State() : time(0, 0, RCL_SYSTEM_TIME) {
        // Robot state keeps the original first six indices for compatibility:
        // [x, y, yaw, v, yaw_rate, gyro_bias_z, z, roll, pitch,
        //  gyro_bias_x, gyro_bias_y].
        int total_size = getRobotStateSize() + getLandmarkStateSize();
        mu = Eigen::VectorXd::Zero(total_size);
        sigma = Eigen::MatrixXd::Identity(total_size, total_size);
        
        // Robot state initial covariance (small uncertainty)
        sigma(0, 0) = 0.01;  // x
        sigma(1, 1) = 0.01;  // y
        sigma(2, 2) = 0.01;  // yaw
        sigma(3, 3) = 0.01;  // v
        sigma(4, 4) = 0.01;  // w
        sigma(5, 5) = (0.5 * M_PI / 180.0) * (0.5 * M_PI / 180.0);  // gyro bias
        sigma(6, 6) = 0.04;  // z
        sigma(7, 7) = std::pow(5.0 * M_PI / 180.0, 2);  // roll
        sigma(8, 8) = std::pow(5.0 * M_PI / 180.0, 2);  // pitch
        sigma(9, 9) = std::pow(0.5 * M_PI / 180.0, 2);  // gyro bias x
        sigma(10, 10) = std::pow(0.5 * M_PI / 180.0, 2);  // gyro bias y
        
        // Landmark initial covariance (large uncertainty, independent of robot)
        // Landmark initialized at (0, 0, 0) in odom frame
        int lm_offset = getRobotStateSize();
        sigma(lm_offset + 0, lm_offset + 0) = 100.0;         // lm_x (10m std)
        sigma(lm_offset + 1, lm_offset + 1) = 100.0;         // lm_y (10m std)
        sigma(lm_offset + 2, lm_offset + 2) = 1.57 * 1.57;   // lm_yaw (90° std)
    }
    
    // Helper function to set state from initial pose message
    void setPose(
        double x,
        double y,
        const std::array<double, 4>& quat,
        double vx,
        double vyaw,
        const std::array<double, 3>& gyro_bias)
    {
        // Only update robot state, keep landmark state unchanged
        mu(0) = x;
        mu(1) = y;
        mu(2) = quat2YawRad(quat);
        mu(3) = vx; // velocity
        mu(4) = vyaw; // angular velocity
        mu(5) = gyro_bias[2];
        mu(9) = gyro_bias[0];
        mu(10) = gyro_bias[1];
        Eigen::Quaterniond eigen_quat(quat[0], quat[1], quat[2], quat[3]);
        if (eigen_quat.squaredNorm() > 1.0e-12) {
            eigen_quat.normalize();
            const Eigen::Matrix3d rotation = eigen_quat.toRotationMatrix();
            mu(7) = std::atan2(rotation(2, 1), rotation(2, 2));
            mu(8) = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
        } else {
            mu(7) = 0.0;
            mu(8) = 0.0;
        }
        // Landmark state remains as initialized.
    }

    static double quat2YawRad(const std::array<double, 4>& quat) {
        double quat_w = quat[0];
        double quat_x = quat[1];
        double quat_y = quat[2];
        double quat_z = quat[3];

        // Convert quaternion to yaw
        Eigen::Quaterniond eigen_quat(quat_w, quat_x, quat_y, quat_z);
        eigen_quat.normalize();  // Ensure unit quaternion

        // Convert to rotation matrix then extract yaw for better numerical stability
        Eigen::Matrix3d rotation_matrix = eigen_quat.toRotationMatrix();

        // Extract yaw from rotation matrix (more stable than direct quaternion formula)
        // yaw = atan2(R21, R11) where R is rotation matrix
        return std::atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));
    }

    // Convert the jointly gyro-propagated roll/pitch/yaw to ROS orientation.
    Eigen::Quaterniond getQuaternion() const {
        const Eigen::AngleAxisd yaw_rotation(mu(2), Eigen::Vector3d::UnitZ());
        const Eigen::AngleAxisd pitch_rotation(mu(8), Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd roll_rotation(mu(7), Eigen::Vector3d::UnitX());
        return Eigen::Quaterniond(yaw_rotation * pitch_rotation * roll_rotation);
    }

    // Get robot state size (position, orientation, velocities, gyro bias)
    static constexpr int getRobotStateSize() { return 11; }
    
    // Get landmark state size (position and orientation)
    static constexpr int getLandmarkStateSize() { return 3; }  // x, y, yaw
    
    // Get total state size (robot + landmarks)
    static constexpr int getTotalStateSize() { return getRobotStateSize() + getLandmarkStateSize(); }
    
};

inline constexpr char INIT_POSE_PROCESSED_EVENT[] = "initial_pose_processed";
inline constexpr char ODOMETRY_UPDATED_EVENT[] = "odometry_processed";
inline constexpr char LANDMARK_PROCESSED_EVENT[] = "landmark_processed";

} // namespace odom_estimation
} // namespace gole

#endif // ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_TYPES_HPP_
