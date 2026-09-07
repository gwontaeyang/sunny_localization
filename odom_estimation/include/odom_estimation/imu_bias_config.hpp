#ifndef ODOM_ESTIMATION_IMU_BIAS_CONFIG_HPP_
#define ODOM_ESTIMATION_IMU_BIAS_CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

namespace gole {
namespace odom_estimation {

inline std::string expandUserPath(const std::string& path)
{
    if (path.empty() || path[0] != '~') {
        return path;
    }

    const char* home = std::getenv("HOME");
    if (home == nullptr || std::strlen(home) == 0) {
        return path;
    }

    if (path.size() == 1) {
        return std::string(home);
    }

    if (path[1] == '/') {
        return std::string(home) + path.substr(1);
    }

    return path;
}

inline std::array<double, 3> loadGyroBiasCalibration(
    const std::string& yaml_path,
    double max_abs_z_rad_s)
{
    if (yaml_path.empty()) {
        throw std::invalid_argument("gyro_bias_yaml_path must not be empty");
    }
    if (!std::isfinite(max_abs_z_rad_s) || max_abs_z_rad_s <= 0.0) {
        throw std::invalid_argument("max_initial_gyro_bias_abs must be finite and greater than zero");
    }

    const std::string resolved_path = expandUserPath(yaml_path);

    try {
        const YAML::Node root = YAML::LoadFile(resolved_path);
        YAML::Node bias_node;

        if (root["imu_calibration"] && root["imu_calibration"]["gyro_bias"]) {
            bias_node = root["imu_calibration"]["gyro_bias"];
        } else if (root["gyro_bias"]) {
            bias_node = root["gyro_bias"];
        }

        if (!bias_node) {
            throw std::runtime_error(
                "IMU calibration YAML has no imu_calibration.gyro_bias or gyro_bias field: " +
                resolved_path);
        }

        std::array<double, 3> bias{};
        if (bias_node.IsSequence()) {
            if (bias_node.size() != bias.size()) {
                throw std::runtime_error(
                    "gyro_bias sequence must contain exactly three values: " + resolved_path);
            }
            for (std::size_t i = 0; i < bias.size(); ++i) {
                bias[i] = bias_node[i].as<double>();
            }
        } else if (bias_node.IsMap()) {
            for (const char* axis : {"x", "y", "z"}) {
                if (!bias_node[axis]) {
                    throw std::runtime_error(
                        std::string("gyro_bias mapping is missing '") + axis + "': " + resolved_path);
                }
            }
            bias = {
                bias_node["x"].as<double>(),
                bias_node["y"].as<double>(),
                bias_node["z"].as<double>()
            };
        } else {
            throw std::runtime_error(
                "gyro_bias must be a mapping or a three-value sequence: " + resolved_path);
        }

        for (double value : bias) {
            if (!std::isfinite(value)) {
                throw std::runtime_error("gyro_bias contains a non-finite value: " + resolved_path);
            }
        }
        for (double value : bias) {
            if (std::abs(value) > max_abs_z_rad_s) {
                throw std::runtime_error(
                    "gyro_bias axis exceeds max_initial_gyro_bias_abs: " + resolved_path);
            }
        }

        return bias;
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "failed to load IMU calibration YAML '" + resolved_path + "': " + error.what());
    }
}

}  // namespace odom_estimation
}  // namespace gole

#endif  // ODOM_ESTIMATION_IMU_BIAS_CONFIG_HPP_
