#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include <eigen3/Eigen/Geometry>

#include "odom_estimation/imu_transform.hpp"
#include "odom_estimation/attitude_propagation.hpp"
#include "odom_estimation/measurement_validation.hpp"
#include "odom_estimation/motion_projection.hpp"

namespace gole {
namespace odom_estimation {
namespace {

geometry_msgs::msg::Quaternion quaternionFromAxisAngle(
    const Eigen::Vector3d& axis,
    double angle)
{
    const Eigen::Quaterniond eigen_quaternion(Eigen::AngleAxisd(angle, axis));
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.w = eigen_quaternion.w();
    quaternion.x = eigen_quaternion.x();
    quaternion.y = eigen_quaternion.y();
    quaternion.z = eigen_quaternion.z();
    return quaternion;
}

TEST(ImuTransformTest, RotatesCompleteAngularVelocityIntoBaseFrame)
{
    geometry_msgs::msg::Vector3 imu_angular_velocity;
    imu_angular_velocity.x = 1.0;
    imu_angular_velocity.y = 0.0;
    imu_angular_velocity.z = 0.0;

    // A +90 degree rotation about base Y maps IMU +X to base -Z.
    const auto rotation = quaternionFromAxisAngle(
        Eigen::Vector3d::UnitY(), M_PI_2);
    const auto base_angular_velocity = rotateAngularVelocity(
        imu_angular_velocity, rotation);

    EXPECT_NEAR(base_angular_velocity.x, 0.0, 1.0e-12);
    EXPECT_NEAR(base_angular_velocity.y, 0.0, 1.0e-12);
    EXPECT_NEAR(base_angular_velocity.z, -1.0, 1.0e-12);
}

TEST(ImuTransformTest, RotatesAngularVelocityCovariance)
{
    const std::array<double, 9> imu_covariance{
        1.0, 0.0, 0.0,
        0.0, 2.0, 0.0,
        0.0, 0.0, 3.0};
    const auto rotation = quaternionFromAxisAngle(
        Eigen::Vector3d::UnitY(), M_PI_2);

    const auto base_covariance = rotateAngularVelocityCovariance(
        imu_covariance, rotation);

    EXPECT_NEAR(base_covariance[0], 3.0, 1.0e-12);
    EXPECT_NEAR(base_covariance[4], 2.0, 1.0e-12);
    EXPECT_NEAR(base_covariance[8], 1.0, 1.0e-12);
}

TEST(ImuTransformTest, RemovesPhysicalMountRotationFromOrientation)
{
    const auto upside_down = quaternionFromAxisAngle(
        Eigen::Vector3d::UnitX(), M_PI);
    const auto base_orientation = transformOrientationToTarget(
        upside_down, upside_down);
    const auto rpy = rollPitchYawFromQuaternion(base_orientation);

    EXPECT_NEAR(rpy[0], 0.0, 1.0e-12);
    EXPECT_NEAR(rpy[1], 0.0, 1.0e-12);
    EXPECT_NEAR(rpy[2], 0.0, 1.0e-12);
}

TEST(ImuTransformTest, PreservesBasePitchWhileIgnoringMountRoll)
{
    const Eigen::Quaterniond world_from_base(
        Eigen::AngleAxisd(-10.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()));
    const Eigen::Quaterniond base_from_sensor(
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    const Eigen::Quaterniond world_from_sensor =
        world_from_base * base_from_sensor;

    geometry_msgs::msg::Quaternion sensor_orientation;
    sensor_orientation.w = world_from_sensor.w();
    sensor_orientation.x = world_from_sensor.x();
    sensor_orientation.y = world_from_sensor.y();
    sensor_orientation.z = world_from_sensor.z();
    const auto mount = quaternionFromAxisAngle(Eigen::Vector3d::UnitX(), M_PI);
    const auto rpy = rollPitchYawFromQuaternion(
        transformOrientationToTarget(sensor_orientation, mount));

    EXPECT_NEAR(rpy[0], 0.0, 1.0e-12);
    EXPECT_NEAR(rpy[1], -10.0 * M_PI / 180.0, 1.0e-12);
}

TEST(AttitudePropagationTest, IntegratesBodyRollRate)
{
    const auto rpy = propagateRpyWithBodyRates(
        0.0, 0.0, 0.0,
        Eigen::Vector3d(10.0 * M_PI / 180.0, 0.0, 0.0), 1.0);

    EXPECT_NEAR(rpy[0], 10.0 * M_PI / 180.0, 1.0e-12);
    EXPECT_NEAR(rpy[1], 0.0, 1.0e-12);
    EXPECT_NEAR(rpy[2], 0.0, 1.0e-12);
}

TEST(AttitudePropagationTest, BodyYawRateCouplesIntoRollAndYawOnSlope)
{
    const double pitch = 20.0 * M_PI / 180.0;
    const double body_yaw_rate = 10.0 * M_PI / 180.0;
    const double dt = 1.0e-5;
    const auto rpy = propagateRpyWithBodyRates(
        0.0, pitch, 0.0,
        Eigen::Vector3d(0.0, 0.0, body_yaw_rate), dt);

    EXPECT_NEAR(
        rpy[0] / dt, std::tan(pitch) * body_yaw_rate, 1.0e-7);
    EXPECT_NEAR(rpy[1], pitch, 1.0e-10);
    EXPECT_NEAR(
        rpy[2] / dt, body_yaw_rate / std::cos(pitch), 1.0e-7);
}

TEST(AttitudePropagationTest, EulerRateMatrixMatchesQuaternionPropagation)
{
    const double roll = -8.0 * M_PI / 180.0;
    const double pitch = 12.0 * M_PI / 180.0;
    const double yaw = 30.0 * M_PI / 180.0;
    const Eigen::Vector3d body_rates(0.03, -0.04, 0.07);
    const double dt = 1.0e-6;
    const auto propagated = propagateRpyWithBodyRates(
        roll, pitch, yaw, body_rates, dt);
    const Eigen::Vector3d expected_rates =
        bodyRatesToEulerRatesMatrix(roll, pitch) * body_rates;

    EXPECT_NEAR(normalizeAngle(propagated[0] - roll) / dt, expected_rates.x(), 1.0e-7);
    EXPECT_NEAR(propagated[1] - pitch, expected_rates.y() * dt, 1.0e-12);
    EXPECT_NEAR(normalizeAngle(propagated[2] - yaw) / dt, expected_rates.z(), 1.0e-7);
}

TEST(MeasurementValidationTest, ZeroCovarianceUsesFallbackAndFloor)
{
    EXPECT_DOUBLE_EQ(covarianceOrFallback(0.0, true, 0.2, 0.01), 0.2);
    EXPECT_DOUBLE_EQ(covarianceOrFallback(0.001, true, 0.2, 0.01), 0.01);
}

TEST(MotionProjectionTest, NoseUpForwardMotionIncreasesRelativeHeight)
{
    const double nose_up_pitch = -10.0 * M_PI / 180.0;
    const auto velocity = projectForwardVelocity(1.0, nose_up_pitch, 0.0);

    EXPECT_NEAR(velocity.x(), std::cos(nose_up_pitch), 1.0e-12);
    EXPECT_NEAR(velocity.y(), 0.0, 1.0e-12);
    EXPECT_GT(velocity.z(), 0.0);
    EXPECT_NEAR(velocity.z(), std::sin(10.0 * M_PI / 180.0), 1.0e-12);
}

TEST(MotionProjectionTest, NoseDownForwardMotionDecreasesRelativeHeight)
{
    const auto velocity = projectForwardVelocity(
        1.0, 10.0 * M_PI / 180.0, M_PI_2);

    EXPECT_NEAR(velocity.x(), 0.0, 1.0e-12);
    EXPECT_GT(velocity.y(), 0.0);
    EXPECT_LT(velocity.z(), 0.0);
}

}  // namespace
}  // namespace odom_estimation
}  // namespace gole
