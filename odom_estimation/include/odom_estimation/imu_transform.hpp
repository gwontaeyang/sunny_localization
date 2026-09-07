#ifndef ODOM_ESTIMATION_IMU_TRANSFORM_HPP_
#define ODOM_ESTIMATION_IMU_TRANSFORM_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

namespace gole {
namespace odom_estimation {

inline Eigen::Matrix3d rotationMatrixFromQuaternion(
    const geometry_msgs::msg::Quaternion& rotation)
{
    Eigen::Quaterniond quaternion(
        rotation.w, rotation.x, rotation.y, rotation.z);
    if (!std::isfinite(quaternion.squaredNorm()) ||
        quaternion.squaredNorm() < 1.0e-12) {
        throw std::invalid_argument("invalid IMU frame rotation quaternion");
    }
    quaternion.normalize();
    return quaternion.toRotationMatrix();
}

inline geometry_msgs::msg::Vector3 rotateAngularVelocity(
    const geometry_msgs::msg::Vector3& angular_velocity,
    const geometry_msgs::msg::Quaternion& target_from_source_rotation)
{
    const Eigen::Vector3d source(
        angular_velocity.x, angular_velocity.y, angular_velocity.z);
    const Eigen::Vector3d target =
        rotationMatrixFromQuaternion(target_from_source_rotation) * source;

    geometry_msgs::msg::Vector3 result;
    result.x = target.x();
    result.y = target.y();
    result.z = target.z();
    return result;
}

inline std::array<double, 9> rotateAngularVelocityCovariance(
    const std::array<double, 9>& covariance,
    const geometry_msgs::msg::Quaternion& target_from_source_rotation)
{
    Eigen::Matrix3d source;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            source(row, column) = covariance[row * 3 + column];
        }
    }

    const Eigen::Matrix3d rotation =
        rotationMatrixFromQuaternion(target_from_source_rotation);
    const Eigen::Matrix3d target = rotation * source * rotation.transpose();

    std::array<double, 9> result{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result[row * 3 + column] = target(row, column);
        }
    }
    return result;
}

// sensor_msgs/Imu orientation represents the source sensor frame in the world
// frame. TF gives target<-source, so world<-target is
// (world<-source) * (source<-target).
inline geometry_msgs::msg::Quaternion transformOrientationToTarget(
    const geometry_msgs::msg::Quaternion& world_from_source_orientation,
    const geometry_msgs::msg::Quaternion& target_from_source_rotation)
{
    Eigen::Quaterniond world_from_source(
        world_from_source_orientation.w,
        world_from_source_orientation.x,
        world_from_source_orientation.y,
        world_from_source_orientation.z);
    Eigen::Quaterniond target_from_source(
        target_from_source_rotation.w,
        target_from_source_rotation.x,
        target_from_source_rotation.y,
        target_from_source_rotation.z);
    if (!std::isfinite(world_from_source.squaredNorm()) ||
        world_from_source.squaredNorm() < 1.0e-12 ||
        !std::isfinite(target_from_source.squaredNorm()) ||
        target_from_source.squaredNorm() < 1.0e-12) {
        throw std::invalid_argument("invalid IMU orientation quaternion");
    }

    world_from_source.normalize();
    target_from_source.normalize();
    Eigen::Quaterniond world_from_target =
        world_from_source * target_from_source.conjugate();
    world_from_target.normalize();

    geometry_msgs::msg::Quaternion result;
    result.w = world_from_target.w();
    result.x = world_from_target.x();
    result.y = world_from_target.y();
    result.z = world_from_target.z();
    return result;
}

inline std::array<double, 3> rollPitchYawFromQuaternion(
    const geometry_msgs::msg::Quaternion& orientation)
{
    Eigen::Quaterniond quaternion(
        orientation.w, orientation.x, orientation.y, orientation.z);
    if (!std::isfinite(quaternion.squaredNorm()) ||
        quaternion.squaredNorm() < 1.0e-12) {
        throw std::invalid_argument("invalid orientation quaternion");
    }
    quaternion.normalize();
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    return {roll, pitch, yaw};
}

}  // namespace odom_estimation
}  // namespace gole

#endif  // ODOM_ESTIMATION_IMU_TRANSFORM_HPP_
