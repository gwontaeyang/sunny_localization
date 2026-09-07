#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

#include "odom_estimation/imu_bias_config.hpp"

namespace gole {
namespace odom_estimation {
namespace {

TEST(ImuBiasConfigPathTest, ExpandsCurrentUsersHome)
{
    const char* home = std::getenv("HOME");
    ASSERT_NE(home, nullptr);

    EXPECT_EQ(
        expandUserPath("~/.ros/imu_calibration.yaml"),
        std::string(home) + "/.ros/imu_calibration.yaml");
}

class ImuBiasConfigTest : public ::testing::Test
{
protected:
    std::string writeYaml(const std::string& contents)
    {
        const std::string path =
            "/tmp/odom_estimation_imu_bias_" + std::to_string(getpid()) + "_" +
            std::to_string(paths_.size()) + ".yaml";
        std::ofstream output(path);
        if (!output.is_open()) {
            throw std::runtime_error("failed to create test YAML: " + path);
        }
        output << contents;
        output.close();
        paths_.push_back(path);
        return path;
    }

    void TearDown() override
    {
        for (const auto& path : paths_) {
            std::remove(path.c_str());
        }
    }

private:
    std::vector<std::string> paths_;
};

TEST_F(ImuBiasConfigTest, LoadsNestedMapping)
{
    const auto path = writeYaml(
        "imu_calibration:\n"
        "  gyro_bias:\n"
        "    x: 0.0001\n"
        "    y: -0.0002\n"
        "    z: -0.0015786434271133833\n");

    const auto bias = loadGyroBiasCalibration(path, 0.05);

    EXPECT_DOUBLE_EQ(bias[0], 0.0001);
    EXPECT_DOUBLE_EQ(bias[1], -0.0002);
    EXPECT_DOUBLE_EQ(bias[2], -0.0015786434271133833);
}

TEST_F(ImuBiasConfigTest, LoadsTopLevelSequence)
{
    const auto path = writeYaml("gyro_bias: [0.001, 0.002, -0.003]\n");

    const auto bias = loadGyroBiasCalibration(path, 0.05);

    EXPECT_DOUBLE_EQ(bias[0], 0.001);
    EXPECT_DOUBLE_EQ(bias[1], 0.002);
    EXPECT_DOUBLE_EQ(bias[2], -0.003);
}

TEST_F(ImuBiasConfigTest, RejectsMissingBias)
{
    const auto path = writeYaml("imu_calibration:\n  frame_id: imu_link\n");

    EXPECT_THROW(loadGyroBiasCalibration(path, 0.05), std::runtime_error);
}

TEST_F(ImuBiasConfigTest, RejectsIncompleteMapping)
{
    const auto path = writeYaml("gyro_bias:\n  x: 0.0\n  y: 0.0\n");

    EXPECT_THROW(loadGyroBiasCalibration(path, 0.05), std::runtime_error);
}

TEST_F(ImuBiasConfigTest, RejectsNonFiniteValue)
{
    const auto path = writeYaml("gyro_bias: [0.0, 0.0, .nan]\n");

    EXPECT_THROW(loadGyroBiasCalibration(path, 0.05), std::runtime_error);
}

TEST_F(ImuBiasConfigTest, RejectsOutOfRangeZBias)
{
    const auto path = writeYaml("gyro_bias: [0.0, 0.0, 0.051]\n");

    EXPECT_THROW(loadGyroBiasCalibration(path, 0.05), std::runtime_error);
}

TEST_F(ImuBiasConfigTest, RejectsOutOfRangeRollBias)
{
    const auto path = writeYaml("gyro_bias: [0.051, 0.0, 0.0]\n");

    EXPECT_THROW(loadGyroBiasCalibration(path, 0.05), std::runtime_error);
}

TEST_F(ImuBiasConfigTest, RejectsMissingFile)
{
    EXPECT_THROW(
        loadGyroBiasCalibration("/tmp/odom_estimation_imu_bias_missing.yaml", 0.05),
        std::runtime_error);
}

}  // namespace
}  // namespace odom_estimation
}  // namespace gole
