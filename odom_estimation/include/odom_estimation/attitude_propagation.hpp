#ifndef ODOM_ESTIMATION_ATTITUDE_PROPAGATION_HPP_
#define ODOM_ESTIMATION_ATTITUDE_PROPAGATION_HPP_

#include <algorithm>
#include <array>
#include <cmath>

#include <eigen3/Eigen/Geometry>

namespace gole {
namespace odom_estimation {

inline double normalizeAngle(const double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

// Integrate a body-frame angular-rate sample into a world<-body orientation.
// The input orientation follows ROS ZYX convention: Rz(yaw) Ry(pitch) Rx(roll).
// Quaternion integration avoids treating body p/q/r as independent Euler rates.
inline std::array<double, 3> propagateRpyWithBodyRates(
    const double roll,
    const double pitch,
    const double yaw,
    const Eigen::Vector3d& body_angular_velocity,
    const double dt)
{
    Eigen::Quaterniond world_from_body(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

    const Eigen::Vector3d rotation_vector = body_angular_velocity * dt;
    const double rotation_angle = rotation_vector.norm();
    Eigen::Quaterniond body_increment = Eigen::Quaterniond::Identity();
    if (rotation_angle > 1.0e-12) {
        body_increment = Eigen::Quaterniond(Eigen::AngleAxisd(
            rotation_angle, rotation_vector / rotation_angle));
    }

    // Angular velocity is expressed in the moving body frame, so compose the
    // increment on the right side of world<-body.
    world_from_body = (world_from_body * body_increment).normalized();
    const Eigen::Matrix3d rotation = world_from_body.toRotationMatrix();
    const double propagated_roll = std::atan2(rotation(2, 1), rotation(2, 2));
    const double propagated_pitch = std::asin(
        std::clamp(-rotation(2, 0), -1.0, 1.0));
    const double propagated_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    return {propagated_roll, propagated_pitch, propagated_yaw};
}

// First-order mapping from body rates [p, q, r] to ZYX Euler rates
// [roll_dot, pitch_dot, yaw_dot]. Ground vehicles stay far from the pitch
// singularity; the floor prevents numerical blow-up on malformed input.
inline Eigen::Matrix3d bodyRatesToEulerRatesMatrix(
    const double roll,
    const double pitch)
{
    const double sin_roll = std::sin(roll);
    const double cos_roll = std::cos(roll);
    const double cos_pitch_raw = std::cos(pitch);
    const double cos_pitch = std::copysign(
        std::max(std::fabs(cos_pitch_raw), 1.0e-6), cos_pitch_raw);
    const double tan_pitch = std::sin(pitch) / cos_pitch;

    Eigen::Matrix3d transform;
    transform <<
        1.0, sin_roll * tan_pitch, cos_roll * tan_pitch,
        0.0, cos_roll,            -sin_roll,
        0.0, sin_roll / cos_pitch, cos_roll / cos_pitch;
    return transform;
}

}  // namespace odom_estimation
}  // namespace gole

#endif  // ODOM_ESTIMATION_ATTITUDE_PROPAGATION_HPP_
