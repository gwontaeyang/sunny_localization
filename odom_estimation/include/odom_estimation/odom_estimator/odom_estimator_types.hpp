#ifndef ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_TYPES_HPP_
#define ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_TYPES_HPP_

#include <string>
#include <stdexcept>
#include <memory>

namespace gole {
namespace odom_estimation {

/**
 * @brief Localization system parameters
 */
struct Params {
    // General localization parameters
    bool use_sim_time = false;
    bool is_forward = true;
    bool using_landmark_correction = false;
    
    // Add more parameter groups here as needed
    // e.g., EKF parameters, AMCL parameters, etc.
};

/**
 * @brief Enumeration of supported odom_estimator types
 */
enum class OdomEstimatorType {
    EKF,        // Extended Kalman Filter
    AMCL,       // Adaptive Monte Carlo Localization  
    PARTICLE,   // Particle Filter
    UKF,        // Unscented Kalman Filter
    SLAM        // Simultaneous Localization and Mapping
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
        case OdomEstimatorType::AMCL:
            return "AMCL";
        case OdomEstimatorType::PARTICLE:
            return "PARTICLE";
        case OdomEstimatorType::UKF:
            return "UKF";
        case OdomEstimatorType::SLAM:
            return "SLAM";
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
    } else if (type_str == "AMCL") {
        return OdomEstimatorType::AMCL;
    } else if (type_str == "PARTICLE") {
        return OdomEstimatorType::PARTICLE;
    } else if (type_str == "UKF") {
        return OdomEstimatorType::UKF;
    } else if (type_str == "SLAM") {
        return OdomEstimatorType::SLAM;
    } else {
        throw std::invalid_argument("Unknown odom_estimator type: " + type_str);
    }
}

struct State
{
    rclcpp::Time time;  // Use ROS2 time directly for better precision
    Eigen::VectorXd mu;
    Eigen::MatrixXd sigma;
    
    // Default constructor
    State() : time(0, 0, RCL_SYSTEM_TIME) {
        // Initialize with robot state + landmark state (6 + 3 = 9 dimensions)
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
        
        // Landmark initial covariance (large uncertainty, independent of robot)
        // Landmark initialized at (0, 0, 0) in odom frame
        int lm_offset = getRobotStateSize();
        sigma(lm_offset + 0, lm_offset + 0) = 100.0;         // lm_x (10m std)
        sigma(lm_offset + 1, lm_offset + 1) = 100.0;         // lm_y (10m std)
        sigma(lm_offset + 2, lm_offset + 2) = 1.57 * 1.57;   // lm_yaw (90° std)
    }
    
    // Constructor with time
    State(const rclcpp::Time& t) : time(t) {
        // Initialize with robot state + landmark state (6 + 3 = 9 dimensions)
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
        
        // Landmark initial covariance (large uncertainty, independent of robot)
        // Landmark initialized at (0, 0, 0) in odom frame
        int lm_offset = getRobotStateSize();
        sigma(lm_offset + 0, lm_offset + 0) = 100.0;         // lm_x (10m std)
        sigma(lm_offset + 1, lm_offset + 1) = 100.0;         // lm_y (10m std)
        sigma(lm_offset + 2, lm_offset + 2) = 1.57 * 1.57;   // lm_yaw (90° std)
    }

    // Helper function to set state from initial pose message
    void setPose(double x, double y, const std::array<double, 4>& quat, double vx, double vyaw)
    {
        // Only update robot state, keep landmark state unchanged
        mu(0) = x;
        mu(1) = y;
        mu(2) = quat2YawRad(quat);
        mu(3) = vx; // velocity
        mu(4) = vyaw; // angular velocity
        mu(5) = 0.0; // gyro bias
        // Landmark state remains as initialized.
    }

    double quat2YawRad(const std::array<double, 4>& quat) {
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

    void setYawRad(const std::array<double, 4>& quat){
        mu(2) = quat2YawRad(quat);
    }

    // Convert yaw (rad) to quaternion using Eigen for numerical stability
    Eigen::Quaterniond getQuaternion() {
        // Create rotation around Z-axis (yaw) using Eigen's stable implementation
        Eigen::AngleAxisd yaw_rotation(mu(2), Eigen::Vector3d::UnitZ());
        return Eigen::Quaterniond(yaw_rotation);
    }

    // Get robot state size (position, orientation, velocities, gyro bias)
    static constexpr int getRobotStateSize() { return 6; }  // x, y, yaw, vx, vyaw, gyro_bias
    
    // Get landmark state size (position and orientation)
    static constexpr int getLandmarkStateSize() { return 3; }  // x, y, yaw
    
    // Get total state size (robot + landmarks)
    static constexpr int getTotalStateSize() { return getRobotStateSize() + getLandmarkStateSize(); }  // 9
    
    // Check if state contains landmarks - now always true
    bool hasLandmarks() const {
        return true;  // Landmarks are always part of the state
    }
    
    // Get number of landmarks - fixed at 1
    int getNumLandmarks() const {
        return 1;  // Always exactly one landmark
    }
    
    // Get landmark state (x, y, yaw) - always returns the single landmark
    Eigen::Vector3d getLandmark(int idx = 0) const {
        int offset = getRobotStateSize() + idx * getLandmarkStateSize();
        return mu.segment<3>(offset);
    }
    
    // Update landmark state by index (default idx=0 for single landmark)
    void updateLandmark(int idx, double lm_x, double lm_y, double lm_yaw) {
        int offset = getRobotStateSize() + idx * getLandmarkStateSize();
        mu(offset + 0) = lm_x;
        mu(offset + 1) = lm_y;
        mu(offset + 2) = lm_yaw;
    }
    
    // Convenience function to update the single landmark without index
    void updateLandmark(double lm_x, double lm_y, double lm_yaw) {
        updateLandmark(0, lm_x, lm_y, lm_yaw);
    }
};

#define INIT_POSE_PROCESSED_EVENT "initial_pose_processed"
#define WHEEL_PROCESSED_EVENT "odometry_processed"
#define LASER_SCAN_PROCESSED_EVENT "laser_scan_processed"
#define POINT_CLOUD_PROCESSED_EVENT "point_cloud_processed"
#define IMU_PROCESSED_EVENT "imu_processed"

} // namespace odom_estimation
} // namespace gole

#endif // ODOM_ESTIMATION_ODOM_ESTIMATOR_ODOM_ESTIMATOR_TYPES_HPP_
