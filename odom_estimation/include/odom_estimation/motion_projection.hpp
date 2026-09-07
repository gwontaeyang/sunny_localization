#ifndef ODOM_ESTIMATION_MOTION_PROJECTION_HPP_
#define ODOM_ESTIMATION_MOTION_PROJECTION_HPP_

#include <cmath>

#include <eigen3/Eigen/Core>

namespace gole {
namespace odom_estimation {

// Project body-frame forward speed into odom using REP-103 Rz(yaw)*Ry(pitch).
// Roll does not change the body X axis. Positive pitch rotates +X toward -Z.
inline Eigen::Vector3d projectForwardVelocity(
    const double forward_speed,
    const double pitch,
    const double yaw)
{
    const double horizontal_speed = forward_speed * std::cos(pitch);
    return Eigen::Vector3d(
        horizontal_speed * std::cos(yaw),
        horizontal_speed * std::sin(yaw),
        -forward_speed * std::sin(pitch));
}

}  // namespace odom_estimation
}  // namespace gole

#endif  // ODOM_ESTIMATION_MOTION_PROJECTION_HPP_
