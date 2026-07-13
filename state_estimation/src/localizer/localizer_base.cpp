#include <eigen3/Eigen/Dense>
#include "logger/logger.hpp"
#include "localization/localizer/localizer_base.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace gole {
namespace localization {

localization::LocalizerBase::LocalizerBase(rclcpp_lifecycle::LifecycleNode* node,
                                            std::shared_ptr<spdlog::logger> logger,
                                            std::shared_ptr<diagnostic::DiagnosticManager> diagnostic_manager,
                                            const Params& params) 
    : node_(node)
    , logger_(logger)
    , diagnostic_manager_(diagnostic_manager)
    , params_(params)
{
    // Initialize state variables
    is_init_.store(false);
    is_doing_state_estimation_.store(false);
    is_landmark_correction_enabled_.store(false);
}

bool LocalizerBase::initialize()
{
    // Declare and get frame ID parameters
    map_frame_id_ = node_->declare_parameter("map_frame_id", map_frame_id_);
    odom_frame_id_ = node_->declare_parameter("odom_frame_id", odom_frame_id_);
    base_link_frame_id_ = node_->declare_parameter("base_link_frame_id", base_link_frame_id_);

    logger_->info("Frame IDs - Map: {}, Odom: {}, Base: {}", map_frame_id_, odom_frame_id_, base_link_frame_id_);

    map_to_odom_transform_.header.frame_id = map_frame_id_;
    map_to_odom_transform_.child_frame_id = odom_frame_id_;
    map_to_odom_transform_.transform.translation.x = 0.0;
    map_to_odom_transform_.transform.translation.y = 0.0;
    map_to_odom_transform_.transform.translation.z = 0.0;
    map_to_odom_transform_.transform.rotation.x = 0.0;
    map_to_odom_transform_.transform.rotation.y = 0.0;
    map_to_odom_transform_.transform.rotation.z = 0.0;
    map_to_odom_transform_.transform.rotation.w = 1.0;

    current_state_.sigma = Eigen::MatrixXd::Zero(State::getTotalStateSize(), State::getTotalStateSize());
    // Set initial covariance values (diagonal matrix)
    // Position uncertainty (x, y) - smaller values for more confident initial pose
    current_state_.sigma(0, 0) = 0.1 * 0.1;  // x variance (0.1 m std)
    current_state_.sigma(1, 1) = 0.1 * 0.1;  // y variance (0.1 m std)
    current_state_.sigma(2, 2) = (10. * M_PI / 180.0) * (10.0 * M_PI / 180.0);  // yaw variance (5° std)
    current_state_.sigma(3, 3) = 0.01;  // v variance
    current_state_.sigma(4, 4) = 0.01;  // w variance
    current_state_.sigma(5, 5) = (0.5 * M_PI / 180.0) * (0.5 * M_PI / 180.0);  // gyro bias variance

    int lm_offset = State::getRobotStateSize();
    current_state_.sigma(lm_offset + 0, lm_offset + 0) = 100.0;
    current_state_.sigma(lm_offset + 1, lm_offset + 1) = 100.0;
    current_state_.sigma(lm_offset + 2, lm_offset + 2) = 1.57 * 1.57;
    

    // Process noise covariance matrices v, w
    Qu_ = Eigen::MatrixXd::Zero(2, 2);
    Qu_ << 0.01,    0.0,
           0.0,     0.01;

    Qr_ = Eigen::MatrixXd::Zero(3, 3);
    Qr_ << 0.01*0.01, 0.0, 0.0,
           0.0, 0.01*0.01, 0.0,
           0.0, 0.0, std::max(2.1536228685297294e-08, 0.001);

    return true;
}


geometry_msgs::msg::TransformStamped LocalizerBase::getOdomToBaselinkTransform() {
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

 State LocalizerBase::getCurrentState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return current_state_;
}

void LocalizerBase::updateStateTime(const rclcpp::Time& new_time) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.time = new_time;
}

std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> LocalizerBase::getLandmarkState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!is_landmark_correction_enabled_.load()) {
        return std::nullopt;
    }
    
    int robot_state_size = State::getRobotStateSize();  // x, y, yaw, v, w, gyro_bias
    
    // Extract landmark state from end of state vector
    // State structure: [robot(6), landmark(3)]
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


nav_msgs::msg::Odometry LocalizerBase::state2Msg() 
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = odom_frame_id_;
    odom.header.stamp = current_state_.time;
    odom.child_frame_id = base_link_frame_id_;
    // Set position from EKF state

    odom.pose.pose.position.x = current_state_.mu(0);
    odom.pose.pose.position.y = current_state_.mu(1);
    odom.pose.pose.position.z = 0.0;  // 2D localization

    // Convert yaw to quaternion
    auto quat = current_state_.getQuaternion();
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    
    odom.twist.twist.linear.x = current_state_.mu(3);  // velocity
    odom.twist.twist.angular.z = current_state_.mu(4) - current_state_.mu(5); // bias-corrected angular velocity

    // Set pose covariance from EKF state covariance
    // ROS covariance: 6x6 matrix [x, y, z, roll, pitch, yaw]
    // EKF state: [x, y, yaw, v, w, gyro_bias] (+ optional landmarks)
    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = 0.0;
    }
    
    // Map EKF covariance to ROS pose covariance
    odom.pose.covariance[0] = current_state_.sigma(0, 0);   // x variance
    odom.pose.covariance[1] = current_state_.sigma(0, 1);   // x-y covariance
    odom.pose.covariance[6] = current_state_.sigma(1, 0);   // y-x covariance
    odom.pose.covariance[7] = current_state_.sigma(1, 1);   // y variance
    odom.pose.covariance[35] = current_state_.sigma(2, 2);  // yaw variance
    
    // Set twist covariance (velocity uncertainties)
    for (int i = 0; i < 36; i++) {
        odom.twist.covariance[i] = 0.0;
    }
    odom.twist.covariance[0] = current_state_.sigma(3, 3);   // linear velocity variance
    odom.twist.covariance[35] = current_state_.sigma(4, 4);  // angular velocity variance

    return odom;
}

void LocalizerBase::updateMapToOdomTransform(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    // Create Eigen transforms for calculation
    Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
    
    map_to_base.translation() = Eigen::Vector3d(
        current_state_.mu(0),
        current_state_.mu(1),
        0.0
    );
    auto map_quat = current_state_.getQuaternion();
    map_to_base.rotate(Eigen::Quaterniond(map_quat.w(), map_quat.x(), map_quat.y(), map_quat.z()));
    
    // Set odometry pose (odom->base_link)
    auto& odom_pos = odom->pose.pose.position;
    auto& odom_quat = odom->pose.pose.orientation;
    odom_to_base.translation() = Eigen::Vector3d(odom_pos.x, odom_pos.y, odom_pos.z);
    odom_to_base.rotate(Eigen::Quaterniond(odom_quat.w, odom_quat.x, odom_quat.y, odom_quat.z));
    
    // Calculate map->odom = map->base_link * inverse(odom->base_link)
    // TODO : Verify correctness of this transformation (Eigen::Isometry3d)
    Eigen::Isometry3d map_to_odom = map_to_base * odom_to_base.inverse();

    updateMapToOdomTransform(map_to_odom, current_state_.time);
}

void LocalizerBase::updateMapToOdomTransform(Eigen::Isometry3d& map_to_odom, rclcpp::Time time) {
    // Calculate map -> odom transform based on corrected pose
    // This should only be called when there's actual localization correction
 
    map_to_odom_transform_.header.stamp = time;
    map_to_odom_transform_.header.frame_id = map_frame_id_;
    map_to_odom_transform_.child_frame_id = odom_frame_id_;
    
    // Set translation
    Eigen::Vector3d translation = map_to_odom.translation();
    map_to_odom_transform_.transform.translation.x = translation.x();
    map_to_odom_transform_.transform.translation.y = translation.y();
    map_to_odom_transform_.transform.translation.z = translation.z();
    
    // Set rotation
    Eigen::Quaterniond rotation(map_to_odom.rotation());
    map_to_odom_transform_.transform.rotation.x = rotation.x();
    map_to_odom_transform_.transform.rotation.y = rotation.y();
    map_to_odom_transform_.transform.rotation.z = rotation.z();
    map_to_odom_transform_.transform.rotation.w = rotation.w();
    
    map_to_odom_initialized_ = true;
    
    logger_->debug("Updated map->odom transform from EKF: tx={:.3f}, ty={:.3f}", 
                  map_to_odom_transform_.transform.translation.x,
                  map_to_odom_transform_.transform.translation.y);
}


void LocalizerBase::updateMapToOdomTransform(Eigen::Vector3d pos, Eigen::Quaterniond angle, rclcpp::Time time) {
    // Calculate map -> odom transform based on corrected pose
    // This should only be called when there's actual localization correction
 
    map_to_odom_transform_.header.stamp = time;
    map_to_odom_transform_.header.frame_id = map_frame_id_;
    map_to_odom_transform_.child_frame_id = odom_frame_id_;
    
    // Set translation
    map_to_odom_transform_.transform.translation.x = pos.x();
    map_to_odom_transform_.transform.translation.y = pos.y();
    map_to_odom_transform_.transform.translation.z = pos.z();
    
    // Set rotation
    map_to_odom_transform_.transform.rotation.x = angle.x();
    map_to_odom_transform_.transform.rotation.y = angle.y();
    map_to_odom_transform_.transform.rotation.z = angle.z();
    map_to_odom_transform_.transform.rotation.w = angle.w();
    
    map_to_odom_initialized_ = true;
    
    logger_->debug("Updated map->odom transform from EKF: tx={:.3f}, ty={:.3f}", 
                  map_to_odom_transform_.transform.translation.x,
                  map_to_odom_transform_.transform.translation.y);
}


} // namespace localization
} // namespace gole
