#include <array>
#include "odom_estimation/odom_estimator/odom_estimator_base.hpp"

namespace gole {
namespace odom_estimation {

odom_estimation::OdomEstimatorBase::OdomEstimatorBase(rclcpp_lifecycle::LifecycleNode* node,
                                            std::shared_ptr<spdlog::logger> logger,
                                            const Params& params) 
    : node_(node)
    , logger_(logger)
    , params_(params)
{
    // Initialize state variables
    is_init_.store(false);
    is_doing_odom_estimation_.store(false);
    is_landmark_correction_enabled_.store(false);
}

bool OdomEstimatorBase::initialize()
{
    // Declare and get frame ID parameters
    odom_frame_id_ = node_->declare_parameter("odom_frame_id", odom_frame_id_);
    base_link_frame_id_ = node_->declare_parameter("base_link_frame_id", base_link_frame_id_);

    logger_->info("Frame IDs - Odom: {}, Base: {}", odom_frame_id_, base_link_frame_id_);

    current_state_.sigma = Eigen::MatrixXd::Zero(State::getTotalStateSize(), State::getTotalStateSize());
    // Set initial covariance values (diagonal matrix)
    // Position uncertainty (x, y) - smaller values for more confident initial pose
    current_state_.sigma(0, 0) = 0.1 * 0.1;  // x variance (0.1 m std)
    current_state_.sigma(1, 1) = 0.1 * 0.1;  // y variance (0.1 m std)
    current_state_.sigma(2, 2) = (10. * M_PI / 180.0) * (10.0 * M_PI / 180.0);  // yaw variance (5° std)
    current_state_.sigma(3, 3) = 0.01;  // v variance
    current_state_.sigma(4, 4) = 0.01;  // w variance
    current_state_.sigma(5, 5) = (0.5 * M_PI / 180.0) * (0.5 * M_PI / 180.0);  // gyro bias variance
    current_state_.sigma(6, 6) = params_.enable_3d_estimation
        ? params_.initial_z_covariance : 0.0;
    current_state_.sigma(7, 7) = params_.enable_3d_estimation
        ? params_.initial_roll_pitch_covariance : 0.0;
    current_state_.sigma(8, 8) = params_.enable_3d_estimation
        ? params_.initial_roll_pitch_covariance : 0.0;
    current_state_.sigma(9, 9) = params_.initial_gyro_bias_covariance;
    current_state_.sigma(10, 10) = params_.initial_gyro_bias_covariance;

    int lm_offset = State::getRobotStateSize();
    current_state_.sigma(lm_offset + 0, lm_offset + 0) = 100.0;
    current_state_.sigma(lm_offset + 1, lm_offset + 1) = 100.0;
    current_state_.sigma(lm_offset + 2, lm_offset + 2) = 1.57 * 1.57;
    

    return true;
}


geometry_msgs::msg::TransformStamped OdomEstimatorBase::getOdomToBaselinkTransform() {
    auto odom_msg = state2Msg();
    geometry_msgs::msg::TransformStamped odom_to_base_link;
    odom_to_base_link.header.stamp = odom_msg.header.stamp;
    odom_to_base_link.header.frame_id = odom_msg.header.frame_id;  // "odom"
    odom_to_base_link.child_frame_id = odom_msg.child_frame_id;    // "base_link"
    
    odom_to_base_link.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_to_base_link.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_to_base_link.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_to_base_link.transform.rotation = odom_msg.pose.pose.orientation;

    return odom_to_base_link;
}

 State OdomEstimatorBase::getCurrentState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> OdomEstimatorBase::getLandmarkState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_landmark_correction_enabled_.load()) {
        return std::nullopt;
    }
    
    int robot_state_size = State::getRobotStateSize();
    
    // Extract landmark state from end of state vector
    // State structure: [robot(11), landmark(3)]
    double lm_x = current_state_.mu(robot_state_size + 0);
    double lm_y = current_state_.mu(robot_state_size + 1);
    double lm_yaw = current_state_.mu(robot_state_size + 2);
    
    geometry_msgs::msg::PoseWithCovarianceStamped landmark_pose;
    landmark_pose.header.stamp = node_->now();
    landmark_pose.header.frame_id = odom_frame_id_;
    
    // Set position
    landmark_pose.pose.pose.position.x = lm_x;
    landmark_pose.pose.pose.position.y = lm_y;
    landmark_pose.pose.pose.position.z = 0.0;
    
    // Convert yaw to quaternion
    Eigen::Quaterniond q(Eigen::AngleAxisd(lm_yaw, Eigen::Vector3d::UnitZ()));
    landmark_pose.pose.pose.orientation.w = q.w();
    landmark_pose.pose.pose.orientation.x = q.x();
    landmark_pose.pose.pose.orientation.y = q.y();
    landmark_pose.pose.pose.orientation.z = q.z();
    
    // Extract covariance from state covariance matrix
    // Landmark covariance is the bottom-right 3x3 block
    // Map to 6x6 covariance (ROS convention: x, y, z, roll, pitch, yaw)
    for (int i = 0; i < 36; i++) {
        landmark_pose.pose.covariance[i] = 0.0;
    }
    
    // Map landmark covariance [x, y, yaw] to pose covariance [x, y, z, roll, pitch, yaw]
    landmark_pose.pose.covariance[0] = current_state_.sigma(robot_state_size + 0, robot_state_size + 0);  // x variance
    landmark_pose.pose.covariance[7] = current_state_.sigma(robot_state_size + 1, robot_state_size + 1);  // y variance
    landmark_pose.pose.covariance[35] = current_state_.sigma(robot_state_size + 2, robot_state_size + 2); // yaw variance
    
    // Also include cross-correlations
    landmark_pose.pose.covariance[1] = current_state_.sigma(robot_state_size + 0, robot_state_size + 1);  // x-y
    landmark_pose.pose.covariance[6] = current_state_.sigma(robot_state_size + 1, robot_state_size + 0);  // y-x
    
    return landmark_pose;
}


nav_msgs::msg::Odometry OdomEstimatorBase::state2Msg() 
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = odom_frame_id_;
    odom.header.stamp = current_state_.time;
    odom.child_frame_id = base_link_frame_id_;
    // Set position from EKF state

    odom.pose.pose.position.x = current_state_.mu(0);
    odom.pose.pose.position.y = current_state_.mu(1);
    odom.pose.pose.position.z = current_state_.mu(6);

    // Convert yaw to quaternion
    auto quat = current_state_.getQuaternion();
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    
    odom.twist.twist.linear.x = current_state_.mu(3);  // velocity
    odom.twist.twist.angular.z = current_state_.mu(4) - current_state_.mu(5); // bias-corrected angular velocity

    // ROS pose covariance order is [x, y, z, roll, pitch, yaw].
    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = 0.0;
    }
    
    constexpr std::array<int, 6> ros_pose_to_state{{0, 1, 6, 7, 8, 2}};
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 6; ++column) {
            odom.pose.covariance[row * 6 + column] = current_state_.sigma(
                ros_pose_to_state[row], ros_pose_to_state[column]);
        }
    }
    
    // Set twist covariance (velocity uncertainties)
    for (int i = 0; i < 36; i++) {
        odom.twist.covariance[i] = 0.0;
    }
    odom.twist.covariance[0] = current_state_.sigma(3, 3);   // linear velocity variance
    odom.twist.covariance[7] = 1.0e6;   // body vy is not directly estimated
    odom.twist.covariance[14] = 1.0e6;  // body vz is not directly estimated
    odom.twist.covariance[21] = 1.0e6;  // roll rate is not an output state
    odom.twist.covariance[28] = 1.0e6;  // pitch rate is not an output state
    odom.twist.covariance[35] = std::max(
        0.0,
        current_state_.sigma(4, 4) + current_state_.sigma(5, 5) -
            2.0 * current_state_.sigma(4, 5));

    return odom;
}

} // namespace odom_estimation
} // namespace gole
